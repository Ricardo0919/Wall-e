import cv2  # type: ignore
import numpy as np  # type: ignore
import time
import paho.mqtt.client as mqtt  # type: ignore
import os

# Configuración MQTT
broker = "10.25.100.90"  # Dirección IP del broker MQTT
port = 1883
object_topic = "esp32/object"  # Tópico para recibir el color del cubo a buscar
movement_topic = "esp32/movement"  # Tópico para enviar la señal de "Stop"
found_object_topic = "esp32/foundObject"  # Tópico para enviar la señal de "Start"

# Inicializa el cliente MQTT
mqtt_client = mqtt.Client()
mqtt_client = mqtt.Client()

# Variable global para el color objetivo
color_objetivo = None
cube_found = False  # Control para evitar mensajes repetidos

# Callback para manejar mensajes del tópico
def on_message(client, userdata, message):
    global color_objetivo, cube_found
    try:
        color_objetivo = message.payload.decode('utf-8').lower()
        if color_objetivo in color_ranges:
            print(f"Color objetivo actualizado a: {color_objetivo}")
            cube_found = False  # Reinicia el estado de búsqueda al cambiar el color
        else:
            print(f"Color recibido '{color_objetivo}' no es válido.")
            color_objetivo = None  # Resetea si no es un color válido
    except Exception as e:
        print(f"Error al procesar el mensaje del tópico {object_topic}: {e}")

# Configuración de cliente MQTT
mqtt_client.on_message = on_message
mqtt_client.connect(broker, port, 60)
mqtt_client.subscribe(object_topic)  # Suscribirse al tópico
mqtt_client.loop_start()

# URL del stream de la cámara DroidCam
droidcam_url = 'http://192.168.137.156:4747/video'  # Cambia la URL según sea necesario

color_ranges = {
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

# Configuración de ventana única
window_name = "ESP32 - Detección y Recogida de Cubos"
cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)  # Crea una única ventana ajustable

def detectar_cubos(img, color_objetivo):
    """Detecta cubos de un color específico en una imagen."""
    global cube_found

    if color_objetivo not in color_ranges:
        print("No hay un color objetivo válido definido.")
        return img

    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    mask_total = None
    for lower, upper in color_ranges[color_objetivo]:
        lower_bound = np.array(lower, dtype=np.uint8)
        upper_bound = np.array(upper, dtype=np.uint8)
        mask = cv2.inRange(hsv, lower_bound, upper_bound)

        if mask_total is None:
            mask_total = mask
        else:
            mask_total = cv2.bitwise_or(mask_total, mask)

    mask_total = cv2.erode(mask_total, None, iterations=2)
    mask_total = cv2.dilate(mask_total, None, iterations=2)
    contours, _ = cv2.findContours(mask_total, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    for contour in contours:
        if cv2.contourArea(contour) < 500:
            continue

        epsilon = 0.04 * cv2.arcLength(contour, True)
        approx = cv2.approxPolyDP(contour, epsilon, True)

        if len(approx) == 4:
            x, y, w, h = cv2.boundingRect(approx)
            aspect_ratio = float(w) / h
            if 0.8 < aspect_ratio < 1.2:
                cv2.drawContours(img, [approx], -1, (0, 255, 0), 2)
                cv2.putText(img, f'Cubo {color_objetivo}', (x, y - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                print(f"Cubo {color_objetivo} detectado en posición ({x}, {y}).")

                if not cube_found:
                    mqtt_client.publish(movement_topic, "Stop")
                    mqtt_client.publish(found_object_topic, "Start")
                    cube_found = True

    return img

try:
    cap = cv2.VideoCapture(droidcam_url)

    while True:
        ret, img = cap.read()
        if not ret or img is None:
            print("No se pudo obtener el frame.")
            time.sleep(1)
            continue

        img = detectar_cubos(img, color_objetivo)
        cv2.imshow(window_name, img)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
except Exception as e:
    print(f"Error durante la ejecución: {e}")
finally:
    cap.release()
    cv2.destroyAllWindows()
    mqtt_client.disconnect()
