import cv2 # type: ignore
import urllib.request
import numpy as np # type: ignore
import time
import paho.mqtt.client as mqtt # type: ignore

# Configuración MQTT
broker = "10.25.100.90"  # Dirección IP del broker MQTT (cambia según sea necesario)
port = 1883
movement_topic = "esp32/movement"
claw_topic = "esp32/claw"

# Inicializa el cliente MQTT
mqtt_client = mqtt.Client()
mqtt_client.connect(broker, port, 60)

# URL del stream de la cámara ESP32 (ajusta la IP a la de tu ESP32)
url = 'http://192.168.137.186/capture'  # Cambia la URL según sea necesario

# Define los rangos de color en HSV para los cubos de color
color_ranges = {
    'rojo': ([0, 82, 182], [13, 162, 255]),         # Rojo
    'azul': ([63, 51, 45], [134, 187, 255]),       # Azul
    'amarillo': ([15, 52, 151], [39, 255, 255])    # Amarillo
}

def detectar_cubos(img, color_ranges):
    """Detecta cubos en una imagen según los rangos de color especificados."""
    # Convertir la imagen a HSV para facilitar la detección de color
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    for color, (lower, upper) in color_ranges.items():
        # Convierte los límites de color a un arreglo de NumPy
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
                    # Calcula la relación entre el área del contorno y el área del bounding box
                    contour_area = cv2.contourArea(contour)
                    bounding_box_area = w * h
                    extent = contour_area / bounding_box_area

                    # Si la relación está dentro de un rango, se considera un cubo
                    if 0.7 < extent < 1.0:  # Ajusta los valores según sea necesario
                        # Dibuja el contorno y etiqueta el cubo detectado
                        cv2.drawContours(img, [approx], -1, (0, 255, 0), 2)
                        cv2.putText(img, f'Cubo {color}', (x, y - 10), 
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                        ##########################################################################################
                        # Detener cualquier movimiento previo y notificar detección
                        mqtt_client.publish(movement_topic, "Stop")
                        print(f"Detectado cubo de color {color}. Iniciando acercamiento.")

                        # Calcular posición relativa del cubo y el centro del marco
                        cx = x + w // 2  # Centro del cubo en el eje x
                        frame_center = img.shape[1] // 2  # Centro del marco en el eje x

                        # Inicializar parámetros de control
                        max_speed = 50  # Velocidad máxima del robot (ajustar según hardware)
                        k_p = 0.1  # Ganancia proporcional para la alineación
                        approach_threshold = 70  # Umbral de área mínima para detenerse
                        min_speed_factor = 0.5  # Factor de reducción mínima de velocidad (por cercanía)

                        # Bucle para ajustar dirección y velocidad dinámicamente
                        while True:
                            # Error de alineación basado en la diferencia entre centros
                            alignment_error = frame_center - cx

                            # Ajustar velocidades proporcionales al error de alineación
                            left_motor_speed = max_speed - k_p * alignment_error
                            right_motor_speed = max_speed + k_p * alignment_error

                            # Normalizar velocidades para estar en el rango permitido
                            left_motor_speed = max(0, min(max_speed, left_motor_speed))
                            right_motor_speed = max(0, min(max_speed, right_motor_speed))

                            # Calcular el factor de reducción según el tamaño del cubo (distancia aparente)
                            bounding_box_area = w * h
                            distance_factor = max(min_speed_factor, min(1.0, 1.0 / bounding_box_area))
                            left_motor_speed *= distance_factor
                            right_motor_speed *= distance_factor

                            # Publicar velocidades calculadas por MQTT
                            mqtt_client.publish(movement_topic, f"LEFT:{int(left_motor_speed)}")
                            mqtt_client.publish(movement_topic, f"RIGHT:{int(right_motor_speed)}")

                            # Detenerse si el cubo ocupa suficiente área en la imagen
                            if bounding_box_area > approach_threshold:
                                print("Robot suficientemente cerca del cubo. Deteniendo movimiento.")
                                mqtt_client.publish(movement_topic, "Stop")
                                break

                            # Esperar un breve intervalo antes de recalcular
                            time.sleep(0.1)

                        # Operar la garra después de detenerse
                        print("Activando garra para recoger el cubo.")
                        mqtt_client.publish(claw_topic, "Open")
                        time.sleep(1)  # Tiempo para abrir la garra
                        mqtt_client.publish(claw_topic, "Close")
                        print("Cubo recogido con éxito.")


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

        # Detectar cubos en la imagen
        img = detectar_cubos(img, color_ranges)

        # Muestra el video en pantalla
        cv2.imshow('ESP32 - Detección y Recogida de Cubos', img)

        # Presiona 'q' para salir
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    except Exception as e:
        print(f"Error al obtener la imagen: {e}")
        time.sleep(1)  # Espera antes de reintentar
        continue

cv2.destroyAllWindows()
mqtt_client.disconnect()
