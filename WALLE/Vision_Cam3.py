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

# Rangos de color en formato HSV para detección de cubos
COLOR_RANGES = {
    'rojo': ([0, 82, 182], [13, 162, 255]),
    'azul': ([63, 51, 45], [134, 187, 255]),
    'amarillo': ([15, 52, 151], [39, 255, 255])
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
