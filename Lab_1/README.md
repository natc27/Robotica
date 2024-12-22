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

En donde se observa la alineación y una rotación de 180° alrededor del eje Y con respecto al sistema coordenado del workobject ##Pastel##.
![Trayectoria de los nombres](Multimedia/Trayectoria_nombres.png)
![Trayectoria de los nombres y la decoración](Multimedia/Trayectoria_todo.png)

Luego, se modificó y codificó en *RAPID*, donde se pueden observar las trayectorias y los movimientos del manipulador para cubrir los puntos esperados. El código se encuentra en una sola trayectoria para todo el nombre con el propósito de mantener las configuraciones coherentes y alcanzables entre sí, sin mebargo la decoración se creó en una trayectoria o path diferente con el propósito de que pueda ser replicable.
Finalmente, utilizando el código en RAPID se incluyeron dos entrada externas, que se accionan con el mando de control físico del robot, una envía el manipulador a una posición para lograr el mantenimiento (colocación y remoción de la herramienta sobre el flanche de motanje) y otra que desarrolla la trayectoria de decoración.
Para estas acciones, en el lenguaje *RAPID* fue elemental utilizar algunas funciones como:
+ MoveL: Controla el movimiento lineal del punto TCP del efector final a un punto establecido o deseado. Establece parámetros de movimiento como punto final, velocidad, zona de acercamiento, objeto de herramienta y objeto de trabajo.
+ MoveC: Crea un movimiento circular por medio de una instrucción de dos puntos, el punto final del arco y un punto por el cual debe pasar la trayectoria, considerando el punto en donde se encuentra anterior a esta instrucción como el punto de inicio del arco. Establece parámetros de movimiento como punto final, velocidad, zona de acercamiento, objeto de herramienta y objeto de trabajo.
+ WaitDI: Crea un tiempo de "espera" hasta que es disparado por un flanco (que puede ser positivo o negativo) en una señal de entrada digital.

Finalmente, el flujo de acciones que describe el código implementados y las operaciones descritas por el manipulador se puede ver sintetizado en el siguiente diagrama de flujo.
![Diagrama de flujo manipulador]()

## Resultados.

Finalmente, el funcionamiento de la rutina de trabajo sobre un plano de 30° en el robot simulado con RobotStudio se puede observar en el siguiente video.
![Video movimiento en plano 30°](Multimedia/Movimiento-inclinado.mp4)
En donde se observa un comportamiento dependiente de las entradas externas como se espera, además a modo de detalle en el siguiente video se puede observar el funcionamientode la trayectoria de los nombres y un caracter de decoración que en este caso es una estrella.
![Video decoración y nombres en plano 30°](Multimedia/Nombres-inclinado.mp4)
Y su implementación en un espacio de trabajo plano se puede observar el mismo movimiento dependiente de las entradas digitales controladas desde el simulador.
![Video movimiento en plano 0°](Multimedia/Movimiento-Plano.mp4)
Con el detalle de la visualización de las trayectorias de decoración en el siguiente archivo.
![Video decoración y nombres en plano 0°](Multimedia/Nombres_plano.mp4)

En todos los casos anteriores es posible visualizar en detalle el comportamiento esperado del manipulador cumpliendo con las condiciones impuestas sobre su desempeño y con resultados exitosos, además de la implementación correcta del módulo de entradas y salidas, que le permite al robot una interacción con el operario de manera inmediata y brinda control sobre las acciones del manipulador para asgurar los entornos de seguridad que requiere la aplicación.