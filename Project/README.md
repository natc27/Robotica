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

## Análisis - Cinemática inversa

Una vez obtenida la matriz de transformación homogénea (MTH), se implementa la función invkin en MATLAB. Esta función calcula las posiciones articulares necesarias para que el efector final del robot alcance una posición deseada en el espacio cartesiano \((x, y, z)\). Recibe como entradas las coordenadas cartesianas, las longitudes de los eslabones (\(l_1, l_2, l_3, l_4\)) y el ángulo de alcance (\(\phi\)).

### Proceso de cálculo

La función utiliza un enfoque geométrico basado en la *ley de senos* y la *ley de cosenos* para determinar las posiciones articulares. El proceso se divide en los siguientes pasos:

1. *Ángulo base (\(q_1\)):*
   \[
   q_1 = \text{rad2deg}(\text{atan2}(y, x)) - 90^\circ
   \]

2. *Posición del efector sin la herramienta:*
   \[
   \text{pwx} = \sqrt{x^2 + y^2} - l_4 \cdot \cos(\phi)
   \]
   \[
   \text{pwz} = z - l_4 \cdot \sin(\phi) - l_1
   \]

3. *Distancia al punto objetivo (\(r\)):*
   \[
   r = \sqrt{\text{pwx}^2 + \text{pwz}^2}
   \]

4. *Ángulo \(q_3\) (Ley de cosenos):*
   \[
   D = \frac{|r^2 - l_2^2 - l_3^2|}{2 \cdot l_2 \cdot l_3}
   \]
   \[
   q_3 = \text{rad2deg}\left(\text{atan2}\left(-\sqrt{1 - D^2}, D\right)\right) + 90^\circ
   \]

5. *Ángulo \(q_2\) (Ley de senos y cosenos):*
   \[
   \alpha = \text{atan2}(\text{pwz}, \text{pwx})
   \]
   \[
   \beta = \text{atan2}\left(l_3 \cdot \sin(q_3 - 90^\circ), l_2 + l_3 \cdot \cos(q_3 - 90^\circ)\right)
   \]
   \[
   q_2 = \text{rad2deg}(\alpha - \beta) - 90^\circ
   \]

6. *Ángulo \(q_4\) (Herramienta):*
   \[
   q_4 = \phi - q_2 - q_3
   \]

7. *Retorno de las posiciones articulares:*
   \[
   q = [q_1, q_2, q_3, q_4]
   \]

### Implementación en MATLAB

```matlab
function q_sim = invkin(x, y, z, l1, l2, l3, l4, phi)
    % Ángulo base
    q1 = rad2deg(atan2(y, x)) - 90;

    % Posición del efector sin la herramienta
    pwx = sqrt(x^2 + y^2) - l4 * cosd(phi);  
    pwz = z - l4 * sind(phi) - l1;  

    % Distancia al punto objetivo
    r = sqrt(pwx^2 + pwz^2);

    % Ángulo q3 (Ley de cosenos)
    D = abs(r^2 - l2^2 - l3^2) / (2 * l2 * l3);
    q3 = rad2deg(atan2(-sqrt(abs(1 - D^2)), D)) + 90;  

    % Ángulo q2 (Ley de senos y cosenos)
    alpha = atan2(pwz, pwx);
    beta = atan2(l3 * sind(q3 - 90), l2 + l3 * cosd(q3 - 90));
    q2 = rad2deg(alpha - beta) - 90;

    % Ángulo de la herramienta
    q4 = phi - q2 - q3;

    % Retornar ángulos articulares
    q_sim = [q1, q2, q3, q4];
end
## Descripción de la solución

La solución fue implementada utilizando ROS2 Humble en Windows, por medio de RoboStack. Previo a la programación, se estudiaron los registros que poseen los servos AX-12A, que son lo utilizados por el robot PincherX-100. Dichos registro se pueden consutal en la tabla de control, que se encuentra en la [página web](https://emanual.robotis.com/docs/en/dxl/ax/ax-12a/#control-table-data-address) del fabricante, y permiten leer y escribir sobre los registros de los dispositivos. Adicionalmente, se utilizó el software Dynamixel Wizard 2, para verificar el estado de los motores.

### Implementación en ROS2 Humble

Al nodo principal del workspace referente al proyecto, se le ha designado el nombre de *phantom_controller* y fue implementado con *ament_python* con el fin de realizar la implementación del controlador en Python. Éste se encarga de comunicarse con los servos, realizando funciones como la activación de torques y la fijación de tanto posiciones como velocidades objetivo. Para ello, se ha importado la libería *dynamixel_sdk* en el archivo correspondiente al nodo. Para comenzar con la configuración de los motores, se instancian dos objetos con clases *PortHandler* y *PacketHandler*, las cuales se encargan de el manejo del puerto en donde esté conectado el conversor U2D2 y de la lectura y escritura de paquetes, respectivamente. 

https://github.com/natc27/Robotica/blob/0de8620d91547bff2e10a3186a70adac67408662/Project/Phantom_ws/src/phantom_controller/phantom_controller/phantom_controller.py#L49-L51

Posteriormente, se utilizan los miembros *openPort*  y *setBaudRate* del objeto *portHandle* para abrir el puerto y fijar la tasa de baudios correspondiente. Para leer datos de los registros, se utiliza la función el miembro *read2ByteTxRx* (o *read4ByteTxRx*, dependiendo del tipo de dato que almacena el registro) del objeto *packetHandler*.

https://github.com/natc27/Robotica/blob/4102386d516958b52753ce6bb3deca950aa36fa6/Project/Phantom_ws/src/phantom_controller/phantom_controller/phantom_controller.py#L167-L173

Es necesario especificar un objeto tipo *PortHandler*, la ID del motor cuyo registro va a ser accedido y la dirección del registro. En este caso, se debe llamar a la función iterativamente para leer las posiciones de todos los motores y almacenarlas en un arreglo.


## Nodos desarrollados en ROS

### Phantom Controller
### Phantom Kinematics
### Joy Mapper
El nodo joy mapper es un nodo que filtra los contenidos del nodo de lectura Joy. El desarrollo se enfoco para el uso de un mando de DualShock4 de PlayStation 4, pero es posible adaptarlo para otros controladores que hagan uso de joysticks.


### DroidCam Publisher
### DroidCam Listener


## Autores
- Oscar Andrés Alvarado.
- Natalia Cely.
- Omar Pérez.
- Julián Pulido.

---
© 2025 - Universidad Nacional de Colombia. Proyecto desarrollado en el curso de Robótica 2024-II.
