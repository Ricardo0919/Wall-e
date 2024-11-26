import cv2
import numpy as np
import time
import paho.mqtt.client as mqtt

# Configuración de MQTT
BROKER_IP = "192.168.137.56"  # Cambia según la dirección IP del broker MQTT
BROKER_PORT = 1883
TOPIC_MOVEMENT = "esp32/movement"
TOPIC_CLAW = "esp32/claw"

# Configuración de la cámara
CAMERA_URL = "http://192.168.137.2:4747/video"  # Cambia según la configuración de tu cámara
WINDOW_NAME = "Detección de Cubos"

# Define los rangos de color en HSV para los cubos de color
color_ranges = {
    'rojo': [
        ([0, 82, 182], [10, 162, 255]),         # Rojo brillante
        ([139, 48, 76], [179, 171, 237]),       # Rojo intenso
        ([170, 82, 182], [180, 162, 255]),      # Rojo oscuro
        ([0, 70, 50], [10, 255, 200]),          # Rojo saturado
        ([160, 60, 100], [180, 200, 255])       # Rojizo
    ],
    'azul': [
        ([63, 51, 45], [90, 187, 255]),         # Azul estándar
        ([35, 87, 112], [146, 221, 217]),       # Azul brillante
        ([91, 51, 45], [134, 187, 255]),        # Azul intenso
        ([100, 150, 50], [120, 255, 255]),      # Azul brillante saturado
        ([85, 50, 70], [110, 220, 230])         # Azul cielo
    ],
    'amarillo': [
        ([15, 52, 151], [30, 255, 255]),        # Amarillo brillante
        ([14, 27, 48], [141, 100, 194]),        # Amarillo pálido
        ([31, 52, 151], [39, 255, 255]),        # Amarillo intenso
        ([20, 70, 100], [35, 220, 250]),        # Amarillo estándar
        ([25, 50, 80], [35, 200, 240])          # Amarillo cálido
    ]
}

# Inicializa el cliente MQTT
def setup_mqtt():
    client = mqtt.Client()
    client.connect(BROKER_IP, BROKER_PORT, 60)
    return client

# Procesa la imagen para detectar cubos según el color
def detectar_cubos(frame, color_ranges):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    for color, (lower, upper) in color_ranges.items():
        lower_bound = np.array(lower, dtype=np.uint8)
        upper_bound = np.array(upper, dtype=np.uint8)

        mask = cv2.inRange(hsv, lower_bound, upper_bound)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for contour in contours:
            if cv2.contourArea(contour) < 500:
                continue

            epsilon = 0.04 * cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, epsilon, True)

            if len(approx) == 4:
                x, y, w, h = cv2.boundingRect(approx)
                aspect_ratio = float(w) / h

                if 0.8 < aspect_ratio < 1.2:  # Verifica si es un cuadrado
                    cv2.drawContours(frame, [approx], -1, (0, 255, 0), 2)
                    cv2.putText(frame, f"Cubo {color}", (x, y - 10), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
    return frame

def main():
    # Configuración inicial
    mqtt_client = setup_mqtt()
    cap = cv2.VideoCapture(CAMERA_URL)

    if not cap.isOpened():
        print("Error: No se pudo abrir el flujo de video.")
        return

    cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                print("Error: No se pudo leer el fotograma.")
                break

            # Detección de cubos
            frame = detectar_cubos(frame, COLOR_RANGES)

            # Mostrar la imagen procesada
            cv2.imshow(WINDOW_NAME, frame)

            # Salir si se presiona 'q'
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    except KeyboardInterrupt:
        print("\nInterrupción manual detectada. Cerrando...")
    finally:
        cap.release()
        cv2.destroyAllWindows()
        mqtt_client.disconnect()
        print("Recursos liberados correctamente.")

if __name__ == "__main__":
    main()
