import cv2  # type: ignore
import urllib.request
import numpy as np  # type: ignore
import time
import paho.mqtt.client as mqtt  # type: ignore

# Configuración MQTT
broker = " 192.168.209.2"  # Dirección IP del broker MQTT
port = 1883
object_topic = "esp32/object"  # Tópico para recibir el color del cubo a buscar

# Inicializa el cliente MQTT
mqtt_client = mqtt.Client()

# Variable global para el color objetivo
color_objetivo = None

# Callback para manejar mensajes del tópico
def on_message(client, userdata, message):
    global color_objetivo
    try:
        color_objetivo = message.payload.decode('utf-8').lower()
        if color_objetivo in color_ranges:
            print(f"Color objetivo actualizado a: {color_objetivo}")
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

# URL del stream de la cámara ESP32 (ajusta la IP a la de tu ESP32)
url = 'http://192.168.209.185/capture'  # Cambia la URL según sea necesario

# Define los rangos de color en HSV para los cubos de color
color_ranges = {
    'rojo': ([139, 48, 76], [179, 171, 237]),  # Rojo
    'azul': ([35, 87, 112], [146, 221, 217]),  # Azul
    'amarillo': ([15, 52, 151], [39, 255, 255])  # Amarillo
}

# Configuración de ventana única
window_name = "ESP32 - Detección y Recogida de Cubos"
cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)  # Ventana única ajustable

def detectar_cubos(img, color_objetivo):
    """Detecta cubos de un color específico en una imagen."""
    # Verifica si hay un color objetivo definido
    if color_objetivo not in color_ranges:
        print("No hay un color objetivo válido definido.")
        return img

    # Convertir la imagen a HSV para facilitar la detección de color
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    # Obtiene los límites del color objetivo
    lower, upper = color_ranges[color_objetivo]
    lower_bound = np.array(lower, dtype=np.uint8)
    upper_bound = np.array(upper, dtype=np.uint8)

    # Crear una máscara para el color específico
    mask = cv2.inRange(hsv, lower_bound, upper_bound)

    # Realiza operaciones morfológicas para mejorar la máscara
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)

    # Encuentra los contornos de las áreas detectadas
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    for contour in contours:
        # Ignora los contornos pequeños que pueden ser ruido
        if cv2.contourArea(contour) < 500:
            continue

        # Aproximar el contorno para detectar polígonos
        epsilon = 0.04 * cv2.arcLength(contour, True)
        approx = cv2.approxPolyDP(contour, epsilon, True)

        # Verifica si el contorno tiene 4 vértices (indica un cuadrilátero)
        if len(approx) == 4:
            # Encuentra el bounding box del contorno
            x, y, w, h = cv2.boundingRect(approx)

            # Verifica la proporción ancho/alto para determinar si es un cuadrado
            aspect_ratio = float(w) / h
            if 0.8 < aspect_ratio < 1.2:  # Aproximadamente cuadrado
                # Dibuja el contorno y etiqueta el cubo detectado
                cv2.drawContours(img, [approx], -1, (0, 255, 0), 2)
                cv2.putText(img, f'Cubo {color_objetivo}', (x, y - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                print(f"Cubo {color_objetivo} detectado en posición ({x}, {y}).")

    return img

while True:
    try:
        # Intenta obtener la imagen de la cámara
        img_resp = urllib.request.urlopen(url)
        img_np = np.array(bytearray(img_resp.read()), dtype=np.uint8)
        img = cv2.imdecode(img_np, -1)

        # Verifica si la imagen se obtuvo correctamente
        if img is None:
            print("No se pudo obtener la imagen.")
            time.sleep(1)  # Espera antes de reintentar
            continue

        # Detectar cubos en la imagen del color objetivo
        img = detectar_cubos(img, color_objetivo)

        # Muestra el video en pantalla
        cv2.imshow('ESP32 - Detección de Cubos', img)

        # Presiona 'q' para salir
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    except Exception as e:
        print(f"Error al obtener la imagen: {e}")
        time.sleep(1)  # Espera antes de reintentar
        continue

cv2.destroyAllWindows()
mqtt_client.disconnect()
