import serial
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# Configuración del puerto serial (ajusta según tu configuración)
SERIAL_PORT = 'COM12'  # Cambia este valor al puerto serial correcto
BAUD_RATE = 115200

# Configuración de la figura 3D
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Dimensiones del cubo
cube_size = 1
cube_vertices = np.array([[-0.5, -0.5, -0.5],
                          [ 0.5, -0.5, -0.5],
                          [ 0.5,  0.5, -0.5],
                          [-0.5,  0.5, -0.5],
                          [-0.5, -0.5,  0.5],
                          [ 0.5, -0.5,  0.5],
                          [ 0.5,  0.5,  0.5],
                          [-0.5,  0.5,  0.5]])

# Función para rotar el cubo usando los ángulos de Euler
def rotate_cube(yaw, pitch, roll):
    # Matrices de rotación para Euler
    R_yaw = np.array([[np.cos(yaw), -np.sin(yaw), 0],
                      [np.sin(yaw), np.cos(yaw), 0],
                      [0, 0, 1]])

    R_pitch = np.array([[np.cos(pitch), 0, np.sin(pitch)],
                        [0, 1, 0],
                        [-np.sin(pitch), 0, np.cos(pitch)]])

    R_roll = np.array([[1, 0, 0],
                       [0, np.cos(roll), -np.sin(roll)],
                       [0, np.sin(roll), np.cos(roll)]])

    # Combinamos las tres rotaciones
    R = np.dot(R_roll, np.dot(R_pitch, R_yaw))

    # Aplicamos la rotación a los vértices del cubo
    rotated_vertices = cube_vertices.dot(R.T)
    return rotated_vertices

# Función para leer los datos del puerto serial
def read_serial_data():
    try:
        # Abre el puerto serial
        with serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1) as ser:
            while True:
                # Lee una línea del puerto serial
                line = ser.readline().decode('utf-8').strip()

                # Verifica que la línea contiene los ángulos de Euler
                if line.startswith("Euler angles:"):
                    # Extrae los valores de Euler
                    parts = line.split('Yaw: ')[1].split(', ')
                    yaw = float(parts[0])
                    pitch = float(parts[1].split('Pitch: ')[1])
                    roll = float(parts[2].split('Roll: ')[1])

                    # Devuelve los valores de Euler
                    return yaw, pitch, roll
    except Exception as e:
        print(f"Error leyendo serial: {e}")
        return None, None, None

# Función de actualización de la animación
def update(frame):
    # Leer los datos de Euler
    yaw, pitch, roll = read_serial_data()

    if yaw is not None and pitch is not None and roll is not None:
        # Rotar el cubo con los ángulos de Euler
        rotated_vertices = rotate_cube(yaw, pitch, roll)

        # Limpiar el gráfico
        ax.cla()

        # Establecer límites de los ejes
        ax.set_xlim(-2, 2)
        ax.set_ylim(-2, 2)
        ax.set_zlim(-2, 2)

        # Dibujar las líneas del cubo
        ax.plot_trisurf(rotated_vertices[:, 0], rotated_vertices[:, 1], rotated_vertices[:, 2], color='b', linewidth=0.2)

        # Títulos y etiquetas
        ax.set_title("Cubo 3D - Orientación en Tiempo Real")
        ax.set_xlabel("X")
        ax.set_ylabel("Y")
        ax.set_zlabel("Z")

# Crear la animación
ani = FuncAnimation(fig, update, interval=50)

# Mostrar la gráfica
plt.show()
