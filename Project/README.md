# Proyecto Final de Robótica - Sistema de Teleoperación

## Descripción del Proyecto
Este proyecto consiste en el desarrollo de una aplicación de teleoperación utilizando el robot **Phantom X Pincher** para realizar tareas de **Pick & Place** en dos modos de operación: 
- **Automático**: con trayectorias preprogramadas.
- **Manual**: mediante un joystick en tiempo real.

El sistema se basa en una conexión **LAN** entre dos computadoras:
- **Zona Local (Maestro)**: donde el operador controla y supervisa la operación.
- **Zona Remota (Esclavo)**: donde se encuentra el robot físico ejecutando los comandos.

La implementación se desarrolla con **ROS2**, utilizando herramientas como **Rviz, CoppeliaSim y el toolbox de robótica de Peter Corke**.

## Requisitos del Sistema
- **Ubuntu 22.04 LTS** con ROS instalado.
- Espacio de trabajo `catkin` o equivalente en Windows.
- **Dynamixel Workbench**: [Repositorio](https://github.com/labsir-un/phantomx-workbench).
- **Paquete Phantom X**: [Repositorio](https://github.com/felipeg17/px_robot).
- **Python o MATLAB 2015b+**.
- **CoppeliaSim EDU**.
- **Joystick físico** para el modo manual.

## Instalación
1. Clonar los repositorios requeridos:
   ```bash
   git clone https://github.com/labsir-un/phantomx-workbench
   git clone https://github.com/felipeg17/px_robot
   ```
2. Instalar dependencias:
   ```bash
   sudo apt update && sudo apt install ros-foxy-dynamixel-sdk
   ```
3. Configurar ROS:
   ```bash
   source /opt/ros/foxy/setup.bash
   export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/catkin_ws/src
   ```
4. Compilar el workspace:
   ```bash
   cd ~/catkin_ws
   catkin_make
   ```
5. Ejecutar la simulación:
   ```bash
   roslaunch px_robot simulation.launch
   ```

## Modo de Operación
### 1. Operación Automática
- Se ejecuta en Rviz y permite seleccionar un punto de destino en la interfaz gráfica.
- El robot sigue trayectorias predefinidas para llevar el material a su destino.
- Se puede activar una cámara virtual para supervisión remota.

### 2. Operación Manual
- Se usa un **joystick** para controlar el robot en tiempo real.
- Se pueden modificar las coordenadas `(x, y, z)` del efector y accionar el **gripper**.
- La velocidad del movimiento es proporcional a la inclinación del joystick.

## Consideraciones Importantes
- Se debe asegurar una **baja latencia** en la red para garantizar un control en tiempo real.
- La simulación en **CoppeliaSim y Rviz** debe reflejar con precisión la dinámica del robot.
- Es recomendable fijar el **Phantom X Pincher** a una base estable para evitar vibraciones o movimientos inesperados.

## Documentación y Reportes
El repositorio contiene la siguiente documentación:
- **Análisis de Cinemática Directa e Inversa** con resultados numéricos.
- **Diagrama de flujo** del sistema.
- **Código en Python/MATLAB** comentado y documentado.
- **Comparación de la teleoperación manual vs automática**.
- **Video de presentación** con la implementación y simulación.

## Autores
- Oscar Andrés Alvarado.
- Natalia Cely.
- Omar Pérez.
- Julián Pulido.

---
© 2024 - Universidad Nacional de Colombia. Proyecto desarrollado en el curso de Robótica 2024-II.
