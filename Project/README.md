# Proyecto Final de Robótica - Sistema de Teleoperación

## Descripción del Proyecto
Este proyecto consiste en el desarrollo de una aplicación de teleoperación utilizando el robot **Phantom X Pincher** para realizar tareas de **Pick & Place** en dos modos de operación: 
- **Automático**: con trayectorias preprogramadas.
- **Manual**: mediante un joystick en tiempo real.

El sistema se basa en una conexión **LAN** entre dos computadoras:
- **Zona Local (Maestro)**: donde el operador controla y supervisa la operación.
- **Zona Remota (Esclavo)**: donde se encuentra el robot físico ejecutando los comandos.

La implementación se desarrolla con **ROS2**, utilizando herramientas como **Matlab, Dynamixel Wizard y diversos paquetes ya creados en ROS2**.

## Análisis - Cinemática directa del Manipulador.
A partir de mediciones se extrajeron los parámetros de Denavit-Hartenberg correspondientes a la cinemática directa del **Phantom X Pincher**. Inicialmente con una representación gráfica basada en mediciones.
![Cinemática Directa](Multimedia/CD-Phantom.jpg)

| $i$ | $d_i$ | $\theta_i$ | $a_i$ | $\alpha_i$ | Offset |
|---|---|---|---|---|---|
| 1 | $L_1$ | $q_1$ | $0$ | $\pi/2$ | $\pi/2$ |
| 2 | $0$ | $q_2$ | $L_2$ | $0$ | $\pi/2$ |
| 3 | $0$ | $q_3$ | $L_3$ | $0$ | $-\pi/2$ |
| 4 | $0$ | $q_4$ | $L_4$ | $0$ | $0$ |

Cuyos valores numéricos son:
$$ L_1= 12 cm; \hspace{0.5cm} L_2= 10.5 cm; \hspace{0.5cm} L_3= 10.5 cm; \hspace{0.5cm} L_4=10 cm$$

Esto a su vez permitió crear el archivo descriptor del robot para que su visualización eventual en entornos como MATLAB, Coppelia y Rviz corresponda al manipulador físico. Los cálculos fueron comparados con las posiciones esperadas, sin embargo, su comprobación final se produce en el control automático.

![Visualización en MATLAB](Multimedia/vis-matlab.jpg)

## Análisis - Cinemática inversa.

Una vez obtenida la matriz de transformación homogénea (MTH) se implementa la función *invkin* en MATLAB, creada para recibir las coordenadas (x,y,z) de una posición deseada, las longitudes de los eslabones, definidas anteriormente, el ángulo de alcance ($\phi$) y el robot para poder realizar la visualización, esta función calcula la distancia del efector final y verifica la alcanzabilidad del mismo, y utilizando el método geométrico (basado en la ley de senos y la ley de cosenos) halla las posiciones articulares en un vector $q=[q_1 q_2 q_3 q_4]$.
El proceso de obtención de las posiciones articulares se realiza de manera recursiva 







## Autores
- Oscar Andrés Alvarado.
- Natalia Cely.
- Omar Pérez.
- Julián Pulido.

---
© 2025 - Universidad Nacional de Colombia. Proyecto desarrollado en el curso de Robótica 2024-II.
