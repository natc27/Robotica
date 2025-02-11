# ROS2 (Robot Operating System 2).

Grupo conformado por:\
**Oscar Andrés Alvarado Buitrago**\
**Natalia Cely Callejas**


A pesar de su nombre, ROS no es un sistema operativo en sí mismo, sino un entorno de trabajo que permite la creación e integración de aplicaciones con robots. Por medio de integraciones modulares permite simular sistemas robóticos, de forma independiente de su Hardware, generando una abstracción sobre los componentes involucrados que se enfoca en su función y el papel que cumplen en la aplicación deseada. 
Para esto, cada uno de los nodos involucrados, entendidos como componentes del sistema, tomará un papel como cliente o publicador y desarrollará actividades relacionadas con servicio, requerimiento o acción.

## Instalación.
Dado que los dos computadores manejan sistema operativo Windows, se realizó inicialmente la instalación de Miniforge a partir de su repositorio en GitHub, agregándolo al PATH predeterminado y registrándolo como base de Python 3.12. Luego, utilizando la terminal de los computadores y siguiendo el tutorial de RoboStack ( *Getting started* ), se realizó la creación del ambiente ROS y se configuraron los canales, añadiéndo canales específicos para RoboStack y eliminando los que se crean por defecto. Luego, utilizando el siguiente comando se ejecutó la instalación de ROS2 en su distribución Humble.

``mamba install ros-humble-desktop``

Dicha instalación se comprobó al acceder a herramientas básicas de ROS2 como *Rviz2* y acceso al nodo *turtlesim*.

En cuanto a los problemas de instalación, fue necesario crear un nuevo usuario en uno de los dispositivos debido a que el usuario principal contaba con un espacio en su nombre, debido a la diferencia en el uso de terminales (Terminal, PowerShell y Conda Terminal) algunas indicaciones, como la creación de una terminal para ejecutar varios nodos simultáneamente fue confusa. Sin embargo, no se presentaron mayores problemas en el momento de las pruebas de instalación. Es importante actualizar las guías, debido a que las indicaciones incluyen el establecimiento de uno de los nodos como maestro por medio del comando `roscore` pero esto solo aplica para ROS1 y no es el caso, por lo que seguir los pasosde la guía puede llevar a errores iniciales o confusiones.

## Control de Turtlesim con Matlab.

a

## Control de Turtlesim con Python.

a

## Control de Turtlesim con comandos ROS.

a

## Uso de Dynamixel Wizard.

aaaa
### Parámetros utilizados.
123

### Comandos utilizados.
mimimi

### Resultados.
fotofotofoto

## Referencias.
![Repositorio Miniforge}(https://github.com/conda-forge/miniforge)
![Robostack|Getting started. Configuracion Mamba ara ROS2](https://robostack.github.io/GettingStarted.html)
