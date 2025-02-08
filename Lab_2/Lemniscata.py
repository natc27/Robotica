from robodk.robolink import *    # API para comunicarte con RoboDK
from robodk.robomath import *    # Funciones matemáticas
import math

#------------------------------------------------
# 1) Conexión a RoboDK e inicialización
#------------------------------------------------
RDK = Robolink()

# Limpiar la pizarra (trayectorias previas)
RDK.Command("DeleteTrace")

# Elegir un robot (si hay varios, aparece un popup)
robot = RDK.ItemUserPick("Selecciona un robot", ITEM_TYPE_ROBOT)
if not robot.Valid():
    raise Exception("No se ha seleccionado un robot válido.")

#------------------------------------------------
# 2) Cargar el Frame donde se quiere dibujar
#------------------------------------------------
frame_name = "Frame_from_Target1"
frame = RDK.Item(frame_name, ITEM_TYPE_FRAME)
if not frame.Valid():
    raise Exception(f'No se encontró el Frame "{frame_name}" en la estación.')

# Asignamos este frame al robot
robot.setPoseFrame(frame)
# Usamos la herramienta activa
robot.setPoseTool(robot.PoseTool())

# Ajustes de velocidad y blending
robot.setSpeed(300)   # mm/s - Aumento de velocidad para mayor fluidez
robot.setRounding(20)  # Mayor blending para eliminar pausas

#------------------------------------------------
# 3) Parámetros de la figura (lemniscata)
#------------------------------------------------
num_points = 720      # Cantidad de puntos para suavidad
A = 150              # Tamaño de la lemniscata (ajustable)
z_surface = 0        # Altura de dibujo
z_safe = 50          # Altura segura para moverse

#------------------------------------------------
# 4) Dibujar la lemniscata con suavidad
#------------------------------------------------
t_max = 2 * math.pi  # Una vuelta completa
b = A / math.sqrt(2)  # Factor de escalado para forma adecuada

for i in range(num_points):
    t = (i / (num_points - 1)) * t_max  # Distribuir puntos uniformemente
    
    # Ecuaciones de la lemniscata de Bernoulli
    denom = math.sin(t)**2 + 1
    x = (A * math.cos(t)) / denom
    y = (A * math.sin(t) * math.cos(t)) / denom
    
    # Movimiento lineal a cada punto sin pausas
    robot.MoveL(transl(x, y, z_surface))

print(f"¡Figura (lemniscata) completada en el frame '{frame_name}'!")