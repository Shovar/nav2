# tb3_navigation

Sistema de navegación autónoma para TurtleBot3 Waffle con A\*, Pure Pursuit y evitación reactiva de obstáculos (ROS 2 Jazzy).

---

## Dependencias

### ROS 2 packages (provistos por el entorno pixi)

| Paquete | Función |
|---|---|
| `rclpy` | Cliente ROS 2 Python |
| `geometry_msgs` | Mensajes TwistStamped, PoseStamped, PointStamped |
| `nav_msgs` | Mensajes Path, OccupancyGrid |
| `sensor_msgs` | Mensajes LaserScan |
| `tf2_ros` | Transform listener y buffer |
| `visualization_msgs` | Mensajes Marker (lookahead point en RViz) |
| `nav2_msgs` | Acción NavigateToPose |
| `nav2_bringup` | Launch de localización (AMCL + map_server) |
| `nav2_common` | Utilidad RewrittenYaml para procesar parámetros |
| `turtlebot3_gazebo` | Mundos Gazebo, modelos URDF/SDF, bridges |
| `ros_gz_sim` | Lanzamiento de Gazebo (Ignition → Gazebo) |
| `ros_gz_bridge` | Bridge entre ROS 2 y Gazebo |
| `ros_gz_image` | Bridge de imagen (cámara) |

### Repositorios TurtleBot3

Clonar dentro de `~/tb3_jazzy_ws/src/`:

| Repositorio | Rama |
|---|---|
| https://github.com/ROBOTIS-GIT/turtlebot3.git | `jazzy` |
| https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git | `jazzy` |
| https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git | `jazzy` |

### Estructura del workspace

```
~/tb3_jazzy_ws/
├── src/
│   ├── turtlebot3/
│   │   ├── tb3_navigation/       ← este módulo (debe estar aquí)
│   │   ├── turtlebot3/
│   │   ├── turtlebot3_msgs/
│   │   └── ... (otros submodules de turtlebot3)
│   └── turtlebot3_simulations/
├── build/
├── install/
└── log/
```

> **Importante:** `tb3_navigation` debe estar dentro de `tb3_jazzy_ws/src/turtlebot3/tb3_navigation/`
> para que `turtlebot3_gazebo` y `nav2_bringup` sean encontrados como dependencias en tiempo de compilación y ejecución.

---

## Instalación

### 1. Crear el workspace y clonar dependencias

```bash
mkdir -p ~/tb3_jazzy_ws/src && cd ~/tb3_jazzy_ws/src

git clone -b jazzy-devel https://github.com/ROBOTIS-GIT/turtlebot3.git
git clone -b jazzy-devel https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
git clone -b jazzy-devel https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
```

### 2. Colocar tb3_navigation

```bash
# Asumiendo que el código está en otro lado, copiarlo o clonarlo dentro del workspace
cp -r /ruta/origen/tb3_navigation ~/tb3_jazzy_ws/src/turtlebot3/tb3_navigation
```

O si está en un repositorio propio:

```bash
git clone <url-de-tu-repo> ~/tb3_jazzy_ws/src/turtlebot3/tb3_navigation
```

### 3. Build

```bash
cd ~/tb3_jazzy_ws
pixi shell -e jazzy        # Activa el entorno ROS 2 Jazzy
colcon build --symlink-install
source install/setup.bash
```

> `--symlink-install` permite modificar Python scripts sin recompilar.

---

## Ejecución

### 1. Definir modelo

```bash
export TURTLEBOT3_MODEL=waffle
```

### 2. Lanzar navegación completa (Gazebo + AMCL + Navigator + RViz)

```bash
ros2 launch tb3_navigation tb3_navigation.launch.py
```

Por defecto usa el mapa `turtlebot3_house_cust` y el mundo `turtlebot3_house`.
El robot se auto-localiza en `(0, 0, 0)` del frame `map`.

### 3. Solo navegación (si Gazebo ya está corriendo)

```bash
ros2 launch tb3_navigation tb3_nav_only.launch.py
```

### 4. Enviar un goal

Opción A — RViz: botón **Nav2 Goal** → click en el mapa.
Opción B — Terminal:

```bash
ros2 topic pub /goal_pose geometry_msgs/PoseStamped \
  "{header: {frame_id: 'map'}, pose: {position: {x: 1.5, y: 0.5, z: 0.0}, orientation: {w: 1.0}}}" \
  --once
```

---

## Arquitectura

Ver `DOCUMENTATION.md` para la descripción completa del sistema.

Resumen:

| Componente | Archivo | Función |
|---|---|---|
| Nodo orquestador | `navigation_node.py` | Action server, TF, suscripciones, timers |
| Planificador global | `astar_planner.py` | A\* sobre occupancy grid con inflado |
| Control local | `local_planner.py` | Pure Pursuit + reactive avoidance |
| Localización | AMCL (vía `nav2_bringup`) | Filtro de partículas con laser + odometría |
| Parámetros | `param/waffle_pure_pursuit.yaml` | Configuración de todos los módulos |
| Launch | `launch/tb3_navigation.launch.py` | Gazebo + localización + navigator + RViz |
| RViz | `rviz/tb3_navigation2.rviz` | Config visual con panel Nav2 |
