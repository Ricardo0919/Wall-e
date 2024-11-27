import cv2
import numpy as np
import time
import paho.mqtt.client as mqtt
import flask_server

# Inicia el servidor Flask
flask_server.iniciar_servidor()
print("Servidor Flask iniciado en http://localhost:5000/video")

# Configuración MQTT
BROKER_IP = "192.168.209.2"  # Dirección IP del broker MQTT
BROKER_PORT = 1883
TOPIC_OBJECT = "esp32/object"  # Tópico para recibir el color del cubo a buscar
TOPIC_MOVEMENT = "esp32/movement"  # Tópico para controlar el movimiento
TOPIC_FOUND_OBJECT = "esp32/foundObject"  # Tópico para indicar detección de cubo
TOPIC_CLAW = "esp32/claw"  # Tópico para controlar la garra

# Inicializa el cliente MQTT
mqtt_client = mqtt.Client()
mqtt_client.connect(BROKER_IP, BROKER_PORT, 60)
mqtt_client.loop_start()

# Variables globales
color_objetivo = None
cube_found = False  # Control para evitar mensajes repetidos
claw_closed = False  # Estado de la garra

# Definición de rangos de colores en HSV
COLOR_RANGES = {
    'rojo': [
        ([0, 82, 182], [10, 162, 255]),
        ([139, 48, 76], [179, 171, 237]),
        ([170, 82, 182], [180, 162, 255]),
        ([0, 70, 50], [10, 255, 200]),
        ([160, 60, 100], [180, 200, 255])
    ],
    'azul': [
        ([63, 51, 45], [90, 187, 255]),
        ([35, 87, 112], [146, 221, 217]),
        ([91, 51, 45], [134, 187, 255]),
        ([100, 150, 50], [120, 255, 255]),
        ([85, 50, 70], [110, 220, 230])
    ],
    'amarillo': [
        ([15, 52, 151], [30, 255, 255]),
        ([14, 27, 48], [141, 100, 194]),
        ([31, 52, 151], [39, 255, 255]),
        ([20, 70, 100], [35, 220, 250]),
        ([25, 50, 80], [35, 200, 240])
    ]
}

# Callback para manejar mensajes del tópico
def on_message(client, userdata, message):
    global color_objetivo, cube_found
    color_objetivo = message.payload.decode('utf-8').lower()
    cube_found = False  # Reinicia el estado al cambiar de color
    print(f"Color objetivo actualizado a: {color_objetivo}")

mqtt_client.on_message = on_message
mqtt_client.subscribe(TOPIC_OBJECT)

# Función para detectar cubos del color objetivo
def detectar_cubos(frame, color_objetivo):
    global cube_found, claw_closed

    if color_objetivo not in COLOR_RANGES:
        print("Color objetivo no válido o no definido.")
        return frame

    # Convertir a HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Crear la máscara combinada para todos los rangos del color objetivo
    mask = None
    for lower, upper in COLOR_RANGES[color_objetivo]:
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
            if 0.8 < aspect_ratio < 1.2:  # Aproximadamente cuadrado
                cv2.drawContours(frame, [approx], -1, (0, 255, 0), 2)
                cv2.putText(frame, f"Cubo {color_objetivo}", (x, y - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                print(f"Cubo {color_objetivo} detectado en posición ({x}, {y}).")

                if not cube_found:
                    mqtt_client.publish(TOPIC_MOVEMENT, "Stop")
                    mqtt_client.publish(TOPIC_FOUND_OBJECT, "Start")
                    print("Movimiento detenido. Cubo encontrado.")
                    cube_found = True
                    time.sleep(2)

                    # Control de la garra
                    if not claw_closed:
                        mqtt_client.publish(TOPIC_CLAW, "Close")
                        print("Garra cerrada.")
                        claw_closed = True
    return frame

# Configuración de la cámara
CAMERA_URL = "http://10.25.82.187:4747/video"
cap = cv2.VideoCapture(CAMERA_URL)

if not cap.isOpened():
    print("No se pudo abrir el flujo de video.")
    exit()

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
