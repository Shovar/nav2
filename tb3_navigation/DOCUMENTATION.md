# Módulo tb3_navigation

Sistema de navegación autónoma para TurtleBot3 Waffle (ROS 2 Jazzy) con planificación global A*, seguimiento Pure Pursuit, evitación reactiva de obstáculos y localización AMCL.

---

## Arquitectura general

```
ROS 2 Jazzy ── tb3_navigation
│
├── navigation_node.py  →  Nodo ROS orquestador (action server, TF, timers)
├── astar_planner.py    →  Planificador global A* sobre occupancy grid
├── local_planner.py    →  Control local: Pure Pursuit + reactive avoidance
└── param/waffle_pure_pursuit.yaml → Parámetros de todo el sistema
```

**Flujo de datos:**

```
Mapa (/map) ──► AstarPlanner (grid + inflado + obstáculos dinámicos)
                    │
                    ▼  (path global)
Goal ──► navigation_node ──► LocalPlanner ──► TwistStamped (/cmd_vel)
           │    ▲                    ▲
           │    │                    │
           ▼    └── TF map→base_link ┘
         AMCL                    LaserScan (/scan)
      (localización)           (obstáculos + avoidance)
```

---

## 1. Localización — AMCL (`nav2_amcl`)

Se lanza desde `localization_launch.py` (nav2_bringup) con los parámetros definidos en `waffle_pure_pursuit.yaml`.

### Configuración

| Parámetro | Valor | Efecto |
|---|---|---|
| `alpha1-5` | 0.05 | Ruido de odometría bajo (Gazebo es preciso) |
| `z_hit / z_rand` | 0.8 / 0.05 | 80% del laser se trata como lectura real, solo 5% como aleatorio |
| `sigma_hit` | 0.3 | Anchura del modelo de impacto del laser |
| `max/min_particles` | 3000 / 1000 | Cobertura del espacio de creencias |
| `update_min_d / _a` | 0.15m / 0.1rad | Frecuencia de actualización del filtro |
| `transform_tolerance` | 0.5s | Tolerancia máxima para transforms TF |
| `set_initial_pose` | true | Auto-localiza al arrancar usando `initial_pose` |

`initial_pose` está en coordenadas del frame `map`. AMCL publica la transformación `map → odom` que permite a los demás nodos convertir coordenadas al frame del mapa.

---

## 2. Planificador global — A* (`astar_planner.py`)

Clase `AstarPlanner` — busca camino óptimo sobre el occupancy grid evitando obstáculos estáticos y dinámicos.

### Mapa

- Se recibe del tópico `/map` (publicado por `map_server`)
- Resolución típica: 0.05 m/celda
- Se almacena internamente junto con origen, dimensiones y resolución

### Inflado de obstáculos

Antes de planificar, se genera una copia inflada del mapa:

- Cada celda ocupada (valor ≥ `obstacle_threshold`, por defecto 60) se expande radialmente `inflation_radius_cells` celdas (actual: 4 celdas = 0.2 m)
- Los obstáculos dinámicos (del laser) también se inflan
- El A* planifica sobre este mapa inflado, nunca sobre el mapa crudo

### Obstáculos dinámicos

- `scan_callback` en `navigation_node.py` proyecta cada punto del laser al frame `map` usando TF
- Se añaden al conjunto `dynamic_obstacles` del planificador
- Se limpian en cada nuevo scan (solo cuenta la lectura más reciente)
- El A* los trata como ocupados e infla alrededor de ellos

### Algoritmo A*

- 8-vecinos (incluyendo diagonales con chequeo de esquina)
- Heurística Euclidiana con peso 1.2
- Costo de celda: `get_cost()` penaliza dinámicos (100) y desconocidos (50)
- Si start/goal caen en ocupado, busca la celda libre más cercana (radio ±20 celdas)
- Devuelve el camino en coordenadas del mundo `[(x,y), ...]`

### Suavizado de camino

```python
smooth_path(path, weight=0.15, passes=1)
```

Una pasada de moving average suave para eliminar el serrucho del grid de 45° sin redondear las esquinas en exceso.

---

## 3. Control local — Pure Pursuit + Avoidance (`local_planner.py`)

Clase `PurePursuitLocalPlanner` — puramente computacional, sin dependencias ROS (solo `math`).

### Entradas

- `set_path(path)`: camino global (lista de waypoints)
- `set_goal(x, y, yaw)`: destino final y orientación deseada
- `compute_velocity(rx, ry, robot_yaw, laser_scan, nearest_obs_dist)`
  - Devuelve: `(vx, wz, goal_reached, lx, ly, avoid_steer, avoid_speed)`

### Pure Pursuit

En cada ciclo de control (30 Hz):

1. **Encontrar índice más cercano** en el camino global a la posición actual del robot
2. **Lookahead point**: primer waypoint a `lookahead_distance` (0.7 m) del robot,
   siguiendo el camino desde el índice más cercano. Si no hay punto a esa distancia,
   se usa el último waypoint.
3. **Ángulo al lookahead**:
   ```
   path_yaw = atan2(ly - ry, lx - rx)
   alpha = normalize_angle(path_yaw - robot_yaw)
   ```
4. **Curvatura y velocidad angular**:
   ```
   curvature = 2 * sin(alpha) / lookahead_distance
   angular_vel = curvature * max_linear_vel
   ```
5. Se limita a `±max_angular_vel` (0.8 rad/s)

### Determinación de velocidad lineal

```
vel = max_linear_vel (0.35 m/s)
```

Se modifica según:

- **Distancia a obstáculos** (desde `nearest_obstacle_distance` del planificador):
  Si `nearest_obs_dist < 0.3` → `vel *= max(0.2, nearest_obs_dist / 0.3)`
- **Aproximación a la meta**:
  Si `dist_to_goal < 0.8 * lookahead_distance` → reducción proporcional hasta `min_linear_vel`
- **Ángulo de giro grande**:
  Si `|alpha| > 60°` → `vel *= 0.4`
- **Avoidance reactivo**:
  `vel *= avoid_speed`

### Reactive avoidance (VFH-like)

Cuando un obstáculo está a menos de `avoid_min_dist` (0.3 m) dentro del cono frontal
(`avoid_forward_angle` = 45°):

1. Se construye un histograma polar de 18 sectores (20° cada uno) con la distancia media
   del laser en cada sector
2. Se ponderan los sectores según su proximidad a la dirección del camino (`rel_path`)
3. Se selecciona el sector con mayor distancia media
4. Se computa un steering `avoid_steer` hacia el centro de ese sector (robot-relative)
5. Se añade a la velocidad angular del Pure Pursuit (mezcla, no reemplazo):
   ```
   angular_vel += avoid_steer * 0.5
   ```
6. Se reduce la velocidad lineal según `avoid_speed` (proporcional a la distancia al obstáculo)

### Suavizado de comandos

```
smooth_vx += 0.12 * (vx - smooth_vx)
smooth_wz += 0.12 * (wz - smooth_wz)
```

Filtro exponencial (alpha=0.12) para evitar cambios bruscos en la salida de velocidad.

---

## 4. Nodo orquestador (`navigation_node.py`)

Clase `PurePursuitNavigator(Node)` — nodo ROS que conecta todos los componentes.

### Responsabilidades

- Declarar y leer parámetros ROS
- Crear `AstarPlanner` y `PurePursuitLocalPlanner`
- Gestionar suscripciones: `/map`, `/scan`, `/goal_pose`
- Publicar: `/cmd_vel` (TwistStamped), `/plan` (Path), `/lookahead_point` (Marker)
- Action server: `/navigate_to_pose` (nav2_msgs/NavigateToPose)
- Timer de control (30 Hz): obtiene pose TF → llama a `LocalPlanner.compute_velocity()` → publica cmd_vel
- Timer de replanificación (0.5 Hz): obtiene pose → llama a `AstarPlanner.plan()` → actualiza camino en LocalPlanner
- Timer de path inicial: reintenta TF hasta 40 veces (4 segundos) antes de abortar

### Estados

| Estado | Descripción |
|---|---|
| `STATE_IDLE` | En espera, sin navegación activa |
| `STATE_COMPUTING_PATH` | Inicial: buscando TF para planificar ruta inicial |
| `STATE_FOLLOWING_PATH` | Siguiendo camino activamente |

### TF tree

```
map  ──(AMCL)──►  odom  ──(Gazebo)──►  base_footprint  ──(URDF)──►  base_link
```

`navigation_node.py` usa `base_link` como `robot_frame` y `map` como `global_frame`.

---

## 5. Parámetros principales (`waffle_pure_pursuit.yaml`)

```
pure_pursuit_navigator:
  lookahead_distance: 0.7        # Distancia de mirada adelante (m)
  max_linear_vel: 0.35           # Velocidad lineal máxima (m/s)
  min_linear_vel: 0.05           # Velocidad lineal mínima (m/s)
  max_angular_vel: 0.8           # Velocidad angular máxima (rad/s)
  goal_tolerance: 0.2            # Tolerancia de posición para goal (m)
  yaw_goal_tolerance: 0.2        # Tolerancia de orientación para goal (rad)
  controller_frequency: 30.0     # Frecuencia del lazo de control (Hz)
  avoidance_min_dist: 0.3        # Distancia mínima para activar avoidance (m)
  avoidance_forward_angle: 45    # Cono frontal para detección (°)
  avoidance_max_speed_red: 0.3   # Factor mínimo de reducción de velocidad
  obstacle_threshold: 60         # Umbral de ocupación del mapa (0-100)
  inflation_radius_cells: 4      # Radio de inflado de obstáculos (celdas)

amcl:
  (ver sección 1)
```

---

## 6. Launch

`tb3_navigation.launch.py` lanza en secuencia:

1. **Gazebo** — mundo + spawn del robot + bridge + state publisher
2. **Localización** — map_server + AMCL + lifecycle_manager (nav2_bringup)
3. **Navigator** — `pure_pursuit_navigator` (tb3_navigation)
4. **RViz** — visualización con panel Nav2

Alternativa: `tb3_nav_only.launch.py` lanza solo localización + navigator + RViz
(sin Gazebo, útil si ya está corriendo).

---

## 7. Resumen del flujo completo

```
1. Gazebo inicia mundo TurtleBot3 + robot en (-2.0, -0.5)
2. map_server carga turtlebot3_house_cust.yaml y publica /map
3. AMCL se inicializa en (0, 0, 0) y comienza a localizar con el laser
4. map_callback() → AstarPlanner recibe el mapa
5. scan_callback() → obstáculos dinámicos proyectados al map frame
6. Usuario envía goal (Nav2 Goal en RViz o /goal_pose)
7. execute_callback() → start_navigation()
8. compute_path_timer_cb():
   a. Espera TF (map → base_link)
   b. AstarPlanner.plan(robot_pose, goal) → path inflado + obstáculos dinámicos
   c. smooth_path(path) → camino suavizado
   d. LocalPlanner.set_path(path) + set_goal(goal)
9. control_loop() cada 33ms:
   a. Obtiene pose robot (TF map → base_link)
   b. LocalPlanner.compute_velocity(pose, scan, obs_dist)
      - Pure Pursuit: lookahead → curvature → angular_vel
      - Avoidance reactivo si obstáculo cercano
      - Suavizado exponencial de vx, wz
   c. Publica TwistStamped en /cmd_vel
10. replan_timer_cb() cada 0.5s: re-planifica con obstáculos actualizados
11. Al alcanzar goal: stop_robot() → goal_handle.succeed()
```
