 # Laboratorio No. 02- Robótica Industrial- Análisis y Operación del Manipulador Motoman MH6: Comparativa y Aplicaciones Prácticas.
 El objetivo del presente laboratorio es entender las similitudes y diferencias entre diferentes modelos y fabricantes de manipuladores en el campo de la robótica industrial.
 En este caso, se inician con las diferencias en las especificaciones técnicas que se presentan entre el Motoman MH6 (Yaskawa) y el IRB140 (ABB).
## Especificaciones técnicas Motoman MH6 vs IRB140.

| Especificación               | Motoman MH6 | IRB140  |
|:----------------------------:|:-----------:|:-------:|
| Imagen de referencia         | ![Motoman MH6](Multimedia/motoman.jpeg) | ![IRB140](Multimedia/IRB140.jpeg)
| Número de ejes               | 6           | 6       |
| Carga máxima                 | 6 kg        | 5 kg    |
| Alcance de la 5 articulación | 1422 mm     | 810 mm  |
| Repitibilidad                | $\pm$ 0.08 mm | $\pm$ 0.03 mm|
| Masa del robot               | 130 kg      | 98 kg   |
| Consumo de potencia          | 1.5 kVA     | 4.5 kVA |
| **Rango de rotación de articulaciones**            |||
| Art. 1                       | 170°        | 360°    |
| Art. 2                       | 155°        | 200°    |
| Art. 3                       | 250°        | 280°    |
| Art. 4                       | 180°        | 400°    |
| Art. 5                       | 225°        | 240°    |
| Art. 6                       | 360°        | 800°    |
| **Velocidad de rotación de articulaciones**        |||
| Art. 1                       | 220°/s      | 200°/s  |
| Art. 2                       | 200°/s      | 200°/s  |
| Art. 3                       | 220°/s      | 260°/s  |
| Art. 4                       | 410°/s      | 360°/s  |
| Art. 5                       | 410°/s      | 360°/s  |
| Art. 6                       | 610°/s      | 450°/s  |
| **Torque máximo en articulaciones**        |||
| Art. 4                       | 11.8 Nm     | 8.58 Nm  |
| Art. 5                       | 9.8 Nm      | 8.58 Nm  |
| Art. 6                       | 5.9 Nm      | 4.91 Nm  |

##  Posiciones de Home para el Manipulador Motoman MH6.
El manipulador Motoman MH6 cuenta con dos posiciones de Home desde fábrica y una tercera denominada Work Home.
*Home 1*. Se trata de la posición inicial y de calibración del manipulador, normalmente no se recomiendo modificar o recalibrar esta posición con excepción de cambio en los encoders o motores. En el laboratorio SalaCAM de la Universidad Nacional de Colombia, esta posición se utiliza para guardar el manipulador en una posición conveniente y "recogida".

![Posición Home 1](Multimedia/Home1.png)
![Home 1 SalaCAM](Multimedia/Home1Lab.jpg)

*Home 2*. Se  puede entender como un checkpoint, es una herramienta para restablecer la posición a una configuración en donde todos los ejes de articulaciones están alineados, en su mayoría en "0". Es posible modificar esta posición posterior a la instalación para recuperarse luego de errores como "Fuera de rango" o singularidades por alineación. En el laboratorio SalaCAM de la Universidad Nacional de Colombia, esta posición se utiliza para establecer la base de movimiento del manipulador, por lo que es una posición lista para iniciar movimiento y trabajo.

![Posición Home 2](Multimedia/Home2.png)
![Home 2 SalaCAM](Multimedia/Home2Lab.jpg)

*Work Home*. Por default se encuentra coincidente con la posición de Home 2, sin embargo presenta también la posibilidad de ser personalizada, su propósito es crear una pocisión que ayuda en las tareas específicas necesaria para mejorar la maniobrabilidad de acuerdo con la aplicación deseada.

![Posición Work Home](Multimedia/WorkHome.png)


## Control del movimiento manual (Jogging).
En la interfaz manual HMI, teach pendant, se encuentran 6 teclas a la izquierda y a la derecha que permiten controlar el movimiento linear del efector final en cada uno de los 3 ejes, cada uno en los sentidos positivo y negativo; si por lo contrario la configuración se encuentra en movimiento articular, las 12 teclas controlaran el movimiento de cada una de las articulaciones en los sentidos positivo y negativo.

![Teclas de movimiento manual](Multimedia/TP.png)

Para realizar el cambio entre el movimiento linear y articular, se utiliza la tecla *MOTION TYPE* en el teach pendant y en la pantalla se observ el tipo de movimiento que se encuentra seleccionado.


## Configuraciones de Velocidad.
El comando *SPEED* permite configurar la velocidad de reproducción de los movimientos programados, admite la diferenciación entre velocidad articular *"VJ=..."*, velocidad del TCP *"V=..."*, velocidad de reprodcción de posición *"VR=..."*, y de reproducción del eje externo *"VE=..."*. Por lo general la confiugración de este parámetro se da en la misma línea de comando que el comando de movimiento con el que se controla el movimiento del manipulador. Para la modificación de este parámetro solo hace falta escoger el valor en el código que se desea modificar y reescribir el nuevo valor.

En la pantalla del Teach Pendant, en la barra superior se encuentra una gráfica de un triángulo dividido en tres, similar al utilizado en dispositivos móviles para reconocer el nivel de señal en el que se encuentra. Esto se puede controlar por medio de las teclas *FAST* y *SLOW* en el centro del teach pendant.

![Nivel de velocidad](Multimedia/Speed.jpg)


## RoboDK.

## Referencias.
+ [Documento con las especificaciones técnicas del Manipulador Motoman MH6.](https://pdf.directindustry.com/pdf/motoman/motoman-mh6-series-robots/14474-97220-_2.html)
+ [Documento con las especificaciones técnicas del Manipulador ABB IRB140.](https://library.e.abb.com/public/73e6655d65ab9569c1257b440052382f/IRB%20140%20datasheet.pdf)
+ [Información del Foro Knowledge Motoman sobre las posiciones de Home](https://knowledge.motoman.com/hc/en-us/articles/4415152176791-Home-Position-Second-Home-Position-and-Work-Home-Position)
+ Manual de referencia para controlador DX100. Motoman.
