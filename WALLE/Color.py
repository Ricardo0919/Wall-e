import cv2
import urllib.request
import numpy as np

# URL del stream de la cámara ESP32
url = 'http://192.168.137.135/capture'  # Cambia la IP si es necesario

# Función vacía para los sliders
def nothing(x):
    pass

# Crear una ventana para los sliders y la imagen procesada
cv2.namedWindow('Configuración HSV y Detección en Tiempo Real', cv2.WINDOW_NORMAL)

# Crear sliders para ajustar los valores HSV
cv2.createTrackbar('H Min', 'Configuración HSV y Detección en Tiempo Real', 0, 179, nothing)
cv2.createTrackbar('H Max', 'Configuración HSV y Detección en Tiempo Real', 179, 179, nothing)
cv2.createTrackbar('S Min', 'Configuración HSV y Detección en Tiempo Real', 0, 255, nothing)
cv2.createTrackbar('S Max', 'Configuración HSV y Detección en Tiempo Real', 255, 255, nothing)
cv2.createTrackbar('V Min', 'Configuración HSV y Detección en Tiempo Real', 0, 255, nothing)
cv2.createTrackbar('V Max', 'Configuración HSV y Detección en Tiempo Real', 255, 255, nothing)

while True:
    try:
        # Intenta obtener la imagen de la cámara
        img_resp = urllib.request.urlopen(url)
        img_np = np.array(bytearray(img_resp.read()), dtype=np.uint8)
        img = cv2.imdecode(img_np, -1)

        # Verifica si la imagen se obtuvo correctamente
        if img is None:
            print("No se pudo obtener la imagen.")
            continue

        # Convertir la imagen a espacio de color HSV
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        # Leer los valores de los sliders
        h_min = cv2.getTrackbarPos('H Min', 'Configuración HSV y Detección en Tiempo Real')
        h_max = cv2.getTrackbarPos('H Max', 'Configuración HSV y Detección en Tiempo Real')
        s_min = cv2.getTrackbarPos('S Min', 'Configuración HSV y Detección en Tiempo Real')
        s_max = cv2.getTrackbarPos('S Max', 'Configuración HSV y Detección en Tiempo Real')
        v_min = cv2.getTrackbarPos('V Min', 'Configuración HSV y Detección en Tiempo Real')
        v_max = cv2.getTrackbarPos('V Max', 'Configuración HSV y Detección en Tiempo Real')

        # Crear límites para el filtro
        lower_bound = np.array([h_min, s_min, v_min], dtype=np.uint8)
        upper_bound = np.array([h_max, s_max, v_max], dtype=np.uint8)

        # Crear la máscara
        mask = cv2.inRange(hsv, lower_bound, upper_bound)

        # Aplicar la máscara a la imagen original
        result = cv2.bitwise_and(img, img, mask=mask)

        # Mostrar la imagen original y la filtrada en la misma ventana
        combined = np.hstack((img, result))
        cv2.imshow('Configuración HSV y Detección en Tiempo Real', combined)

        # Presiona 'q' para salir del bucle
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    except Exception as e:
        print(f"Error al obtener la imagen: {e}")
        continue

# Cerrar todas las ventanas abiertas al finalizar
cv2.destroyAllWindows()