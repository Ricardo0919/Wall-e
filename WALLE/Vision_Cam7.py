import cv2
import numpy as np
import paho.mqtt.client as mqtt
import flask_server

flask_server.iniciar_servidor()
print("Servidor Flask iniciado en http://localhost:5000/video")


# Configuración MQTT
broker = "192.168.5.2"  # Dirección IP del broker MQTT
port = 1883
object_topic = "esp32/object"  # Tópico para recibir el color del cubo a buscar
movement_topic = "esp32/movement"  # Tópico para enviar la señal de "Stop"
found_object_topic = "esp32/foundObject"  # Tópico para enviar la señal de "Start"
position_x_topic = "esp32/objectCenterX"  # Tópico para enviar centro de x

# Inicializa el cliente MQTT
mqtt_client = mqtt.Client()
mqtt_client.connect(broker, port, 60)
mqtt_client.loop_start()

# Variable global para el color objetivo
color_objetivo = None
cube_found = False  # Control para evitar mensajes repetidos

# Callback para manejar mensajes del tópico
def on_message(client, userdata, message):
    global color_objetivo, cube_found
    color_objetivo = message.payload.decode('utf-8').lower()
    cube_found = False  # Reinicia el estado al cambiar de color
    print(f"Color objetivo actualizado a: {color_objetivo}")

mqtt_client.on_message = on_message
mqtt_client.subscribe(object_topic)

# Definición de rangos de colores en HSV
color_ranges = {
    'rojo': [
        ([118, 163, 44], [179, 255, 255])       # Rojo
    ],
    'azul': [
        ([104, 34, 0], [115, 255, 221])         # Azul
    ],
    'amarillo': [
        ([16, 140, 148], [29, 255, 255])        # Amarillo
    ]
}

# Fuente de video
url = "http://192.168.5.8:4747/video"  # Ajusta la URL según tu configuración
cap = cv2.VideoCapture(url)

if not cap.isOpened():
    print("No se pudo abrir el flujo de video.")
    exit()

def detectar_cubos(frame, color_objetivo):
    """Detecta cubos del color objetivo en el frame proporcionado."""
    global cube_found

    if color_objetivo not in color_ranges:
        print("Color objetivo no válido o no definido.")
        return frame

    # Convertir a HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Crear la máscara combinada para todos los rangos del color objetivo
    mask = None
    for lower, upper in color_ranges[color_objetivo]:
        lower_bound = np.array(lower, dtype=np.uint8)
        upper_bound = np.array(upper, dtype=np.uint8)
        temp_mask = cv2.inRange(hsv, lower_bound, upper_bound)
        mask = temp_mask if mask is None else cv2.bitwise_or(mask, temp_mask)

    # Filtrar ruido
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)

    # Detectar contornos
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    for contour in contours:
        if cv2.contourArea(contour) < 500:
            continue

        # Detectar contornos cuadrados
        epsilon = 0.04 * cv2.arcLength(contour, True)
        approx = cv2.approxPolyDP(contour, epsilon, True)

        if len(approx) == 4:  # Aproximadamente un cuadrado
            x, y, w, h = cv2.boundingRect(approx)
            aspect_ratio = float(w) / h
            if 0.2 < aspect_ratio < 1.8:  # Aproximadamente cuadrado
                
                contour_area = cv2.contourArea(contour)
                bounding_box_area = w * h
                extent = contour_area / bounding_box_area

                if 0.6 < extent < 1.1:

                    cv2.drawContours(frame, [approx], -1, (0, 255, 0), 2)
                    cv2.putText(frame, f"Cubo {color_objetivo}", (x, y - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                    print(f"Cubo {color_objetivo} detectado en posición ({x}, {y}).")

                    # Calcula el centro del cubo
                    center_x = x + w // 2
                    center_y = y + h // 2

                    # Calcula el centro del frame
                    frame_center_x = frame.shape[1] // 2
                    frame_center_y = frame.shape[0] // 2

                    # Calcula la distancia entre el centro del cubo y el centro del frame
                    distance_x = center_x - frame_center_x
                    distance_y = center_y - frame_center_y

                    # Imprime la información en consola
                    print(f"Cubo {color_objetivo} detectado en posición centro ({center_x}, {center_y}).")
                    print(f"Distancia al centro del frame: X = {distance_x}, Y = {distance_y}")

                    mqtt_client.publish(position_x_topic, str(distance_x))

                    if not cube_found:
                        mqtt_client.publish(movement_topic, "Stop")
                        mqtt_client.publish(found_object_topic, "Start")
                        cube_found = True

    return frame

# Loop principal
try:
    while True:
        ret, frame = cap.read()
        if not ret:
            print("No se pudo leer el fotograma.")
            break

        frame = detectar_cubos(frame, color_objetivo)
        cv2.imshow("Detección de Cubos", frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
except KeyboardInterrupt:
    print("Ejecución interrumpida.")
finally:
    cap.release()
    cv2.destroyAllWindows()
    mqtt_client.loop_stop()
    mqtt_client.disconnect()
