
# Robulab Control 🦾

Este paquete ROS1 permite el control del robot Robulab10 mediante gestos de mano capturados por cámara USB, utilizando visión por computadora con MediaPipe y OpenCV. El sistema está desarrollado para Ubuntu 20.04 con ROS Noetic, ejecutado dentro de un contenedor Docker personalizado con soporte CUDA.

## Estructura del proyecto

```
catkin_ws/
├── src/
│   └── robulab_control/
│       ├── CMakeLists.txt
│       ├── package.xml
│       ├── launch/
│       │   └── control_gestos.launch
│       ├── scripts/
│       │   └── gesture_node.py
│       └── src/
```

## Requisitos

Asegúrate de tener instaladas las siguientes herramientas y librerías en tu entorno (Docker o nativo):

- Ubuntu 20.04
- ROS Noetic
- Python 3.8+
- OpenCV (`cv2`)
- MediaPipe
- PyTorch + CUDA (si se usa con YOLOv8 u otros modelos)
- numpy
- pandas
- sympy
- nano
- gedit

## Instalación

1. Clona el repositorio dentro de tu espacio de trabajo catkin:
   ```bash
   cd ~/catkin_ws/src
   git clone https://github.com/AlejandroSIRob/Robulab_Control.git robulab_control
   ```

2. Compila el paquete:
   ```bash
   cd ~/catkin_ws
   catkin_make
   source devel/setup.bash
   ```

3. Dale permisos de ejecución al script:
   ```bash
   chmod +x ~/catkin_ws/src/robulab_control/scripts/gesture_node.py
   ```

## Uso

Puedes ejecutar el nodo principal que detecta gestos y publica órdenes de movimiento usando:

```bash
roslaunch robulab_control control_gestos.launch
```

Este nodo detecta los siguientes gestos mediante la cámara:

| Dedos levantados | Acción             |
|------------------|--------------------|
| 0                | MODO AUTOMÁTICO    |
| 1 (índice)       | GIRAR              |
| 2                | IZQUIERDA          |
| 3                | DERECHA            |
| 5                | PARAR              |

> ⚠️ En **modo automático**, el robot deja de aceptar órdenes excepto la de PARAR, que debe mantenerse durante al menos 10 fotogramas seguidos para desactivarlo.

## Notas adicionales

- El nodo `gesture_node.py` puede adaptarse fácilmente para publicar a `/cmd_vel` si se integra con `rospy` y `geometry_msgs/Twist`.
- Se recomienda probar con una cámara USB conectada al host y mapeada dentro del contenedor con `--device=/dev/video0`.

## Contacto

Desarrollado por [AlejandroSIRob](https://github.com/AlejandroSIRob)
