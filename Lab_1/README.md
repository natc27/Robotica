# Laboratorio 1


Grupo conformado por:\
**Oscar Andrés Alvarado Buitargo**\
**Natalia Cely Callejas**

## Diseño de la herramienta.

En el software *Autodesk Inventor* se diseñó el modelo 3D de la herramienta que sostendría el marcador, para lo cual se inició tomando las medidas de un marcador Expo genérico y creando la herramienta manteniendo los ejes de la base coincidentes con los ejes del flanche de montaje en RobotStudio, lo que permitió una simplificación en la unión entre los dos.\
![Herramienta Diseñada en Inventor](Multimedia/Herramienta.jpeg)

Esta herramienta sigue las recomendaciones de un sistea de compensación por lo que se creó en dos piezas roscadas entre sí. Una base que sostendría el marcador y una tapa que contiene un sistema de resorte y un segundo fondo que permita que el marcador se ajuste a la posición y fuerza que ejerce el robot en cada momento.

## Diseño de las letras.

Para continuar, se creó un boceto en *Autodesk Inventor* con los nombres de los integrantes del grupo, separados como se solicitó, y se realizó su extrusión con 100 mm de profundidad. Para permitir un acercamiento con el manipulador y poder crear su trayectoria, se importó la geometria a RobotStudio y se unió a una plataforma en un ángulo de 30° sobre la cual se posicionó un cilindro que simulará la posición del pastel con un diámetro de 25 cm puesto que se proyecta para 25 personas.
![Letras Nombres](Multimedia/Nombres.jpeg)
![Superficie de Trabajo](Multimedia/Superficie.png)

Así, la escena total en RobotStudio incluyendo el manipulador, el controlador virtual, la herramienta o efector final, superficie de trabajo (pastel), mesa y las letras a recorrer se observan en la siguiente imagen.
![Estación de Trabajo](Multimedia/Station.png)

Para esto, la base del manipulador se encuentra en las coordenadas (-269.17,-419.57,0), la mesa de trabajo se encuentra en la posición (230.25,-345.35,0) y su orientación se describe por medio de cuaterniones de la forma (0.7071,0,0,-0.7071) y las letras cuentan con una posición de (355.25,-391.38,230.95) y la orientación descrita por cuaterniones es (0.6830,0.1830,-0.1830,-0.6830).

## Diseño de las trayectorias.

Una vez todos los elementos necesarios se encontraban en la escena de RobotStudio, se inició a programar los diferentes puntos objetivo o targets de las letras por medio de la creación de targets por tabla.
![Targets creados](Targets.png)

En donde se observa

*RAPID* las trayectorias y los movimientos del manipulador para cubrir los puntos esperados. Para optimizar el código, considerando las letras repetidas, se crearon diversos WorkObjects para las letras, por lo que las trayectorias son subrutinas que se pueden replicar más adelante.

## Resultados.

Finalmente, el funcionamiento de la rutina de trabajo en el robot simulado con RobotStudio se puede observar en el siguiente video.
Y su implementación en el manipulador real se puede observar en el siguiente video, en donde se evidencia un comportamiento muy cercano y que cumple con los requerimientos de desempeño esperados.

## Entregables.
El flujo de acciones del robot se puede observar...
