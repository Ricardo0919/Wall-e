import cv2
import urllib.request
import numpy as np
import time

# URL del stream de la cámara ESP32 (ajusta la IP a la de tu ESP32)
url = 'http://192.168.137.3/capture'  # Cambia la URL según sea necesario

# Define los rangos de color en HSV para diferentes colores de pelota
color_ranges = {
    'rojo': ([0, 120, 70], [10, 255, 255]),         # Rojo (ajusta para obtener el color deseado)
    'verde': ([35, 100, 100], [85, 255, 255]),      # Verde
    'azul': ([100, 150, 0], [140, 255, 255])        # Azul
}

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
                if cv2.contourArea(contour) < 150:
                    continue

                # Segundo filtro: Verificar si la figura es circular
                # Encuentra el área y el perímetro del contorno
                area = cv2.contourArea(contour)
                perimeter = cv2.arcLength(contour, True)
                
                # Calcular el "circularity" o circularidad
                # Circularidad = 4 * pi * (area / perimeter^2)
                if perimeter == 0:
                    continue
                circularity = 4 * np.pi * (area / (perimeter * perimeter))
                
                # Si la circularidad está cerca de 1, la forma es aproximadamente un círculo
                if 0.7 < circularity < 1.5:
                    # Encuentra el contorno y dibuja un círculo alrededor de la pelota detectada
                    (x, y), radius = cv2.minEnclosingCircle(contour)
                    center = (int(x), int(y))
                    radius = int(radius)
                    cv2.circle(img, center, radius, (0, 255, 0), 2)
                    cv2.putText(img, f'Pelota {color}', (int(x) - 10, int(y) - 10), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        # Muestra el video en pantalla
        cv2.imshow('ESP32 - Detección de Pelotas de Colores y Forma Circular', img)

        # Presiona 'q' para salir
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    except Exception as e:
        print(f"Error al obtener la imagen: {e}")
        time.sleep(1)  # Espera antes de reintentar
        continue

cv2.destroyAllWindows()
