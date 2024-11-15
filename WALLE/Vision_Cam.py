import cv2
import urllib.request
import numpy as np
import time

# URL del stream de la cámara ESP32
url = 'http://192.168.137.3/capture'  # Cambia la IP si es necesario

# Rangos de color en HSV
color_ranges = {
    'azul': ([100, 150, 0], [140, 255, 255]),
    'rojo': ([0, 120, 70], [10, 255, 255]),
    'amarillo': ([20, 100, 100], [30, 255, 255]),
    'verde': ([35, 100, 100], [85, 255, 255])  # Rango para verde
}

# Crear una ventana para cada color
cv2.namedWindow('ESP32 - Video en Vivo', cv2.WINDOW_NORMAL)
cv2.namedWindow('Detección de Azul', cv2.WINDOW_NORMAL)
cv2.namedWindow('Detección de Rojo', cv2.WINDOW_NORMAL)
cv2.namedWindow('Detección de Amarillo', cv2.WINDOW_NORMAL)
cv2.namedWindow('Detección de Verde', cv2.WINDOW_NORMAL)

while True:
    try:
        # Intenta obtener la imagen de la cámara
        img_resp = urllib.request.urlopen(url)
        img_np = np.array(bytearray(img_resp.read()), dtype=np.uint8)
        img = cv2.imdecode(img_np, -1)

        # Verifica si la imagen se obtuvo correctamente
        if img is None:
            print("No se pudo obtener la imagen.")
            time.sleep(1)
            continue

        # Convertir la imagen a espacio de color HSV
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        # Crear máscaras para cada color
        masks = {}
        for color, (lower, upper) in color_ranges.items():
            lower_bound = np.array(lower, dtype=np.uint8)
            upper_bound = np.array(upper, dtype=np.uint8)
            masks[color] = cv2.inRange(hsv, lower_bound, upper_bound)

        # Mostrar la imagen original y las máscaras para cada color
        cv2.imshow('ESP32 - Video en Vivo', img)
        cv2.imshow('Detección de Azul', masks['azul'])
        cv2.imshow('Detección de Rojo', masks['rojo'])
        cv2.imshow('Detección de Amarillo', masks['amarillo'])
        cv2.imshow('Detección de Verde', masks['verde'])

        # Presiona 'q' para salir del bucle
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    except Exception as e:
        print(f"Error al obtener la imagen: {e}")
        time.sleep(1)
        continue

# Cerrar todas las ventanas abiertas al finalizar
cv2.destroyAllWindows()
