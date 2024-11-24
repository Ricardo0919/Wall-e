import cv2
import urllib.request
import numpy as np

# URL del stream de la cámara ESP32
url = 'http://192.168.209.185/capture'  # Cambia la IP si es necesario

# Función vacía para los sliders
def nothing(x):
    pass

# Crear una ventana para los sliders y la imagen procesada
try:
    cv2.namedWindow('Configuración HSV y Detección en Tiempo Real', cv2.WINDOW_NORMAL)

    # Crear sliders para ajustar los valores HSV
    cv2.createTrackbar('H Min', 'Configuración HSV y Detección en Tiempo Real', 0, 179, nothing)
    cv2.createTrackbar('H Max', 'Configuración HSV y Detección en Tiempo Real', 179, 179, nothing)
    cv2.createTrackbar('S Min', 'Configuración HSV y Detección en Tiempo Real', 0, 255, nothing)
    cv2.createTrackbar('S Max', 'Configuración HSV y Detección en Tiempo Real', 255, 255, nothing)
    cv2.createTrackbar('V Min', 'Configuración HSV y Detección en Tiempo Real', 0, 255, nothing)
    cv2.createTrackbar('V Max', 'Configuración HSV y Detección en Tiempo Real', 255, 255, nothing)
except cv2.error as e:
    print(f"No se pudo crear la ventana: {e}")
    print("Continuando sin mostrar las imágenes en tiempo real.")

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
        h_min = cv2.getTrackbarPos('H Min', 'Configuración HSV y Detección en Tiempo Real') if 'Configuración HSV y Detección en Tiempo Real' in cv2.getWindowProperty else 0
        h_max = cv2.getTrackbarPos('H Max', 'Configuración HSV y Detección en Tiempo Real') if 'Configuración HSV y Detección en Tiempo Real' in cv2.getWindowProperty else 179
        s_min = cv2.getTrackbarPos('S Min', 'Configuración HSV y Detección en Tiempo Real') if 'Configuración HSV y Detección en Tiempo Real' in cv2.getWindowProperty else 0
        s_max = cv2.getTrackbarPos('S Max', 'Configuración HSV y Detección en Tiempo Real') if 'Configuración HSV y Detección en Tiempo Real' in cv2.getWindowProperty else 255
        v_min = cv2.getTrackbarPos('V Min', 'Configuración HSV y Detección en Tiempo Real') if 'Configuración HSV y Detección en Tiempo Real' in cv2.getWindowProperty else 0
        v_max = cv2.getTrackbarPos('V Max', 'Configuración HSV y Detección en Tiempo Real') if 'Configuración HSV y Detección en Tiempo Real' in cv2.getWindowProperty else 255

        # Crear límites para el filtro
        lower_bound = np.array([h_min, s_min, v_min], dtype=np.uint8)
        upper_bound = np.array([h_max, s_max, v_max], dtype=np.uint8)

        # Crear la máscara
        mask = cv2.inRange(hsv, lower_bound, upper_bound)

        # Aplicar la máscara a la imagen original
        result = cv2.bitwise_and(img, img, mask=mask)

        try:
            # Mostrar la imagen original y la filtrada en la misma ventana
            combined = np.hstack((img, result))
            cv2.imshow('Configuración HSV y Detección en Tiempo Real', combined)

            # Presiona 'q' para salir del bucle
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        except cv2.error as e:
            print(f"No se pudo mostrar la imagen: {e}")

        # Guardar imágenes si las ventanas no funcionan
        cv2.imwrite('original.jpg', img)
        cv2.imwrite('result.jpg', result)
        print("Imagen procesada y guardada.")

    except Exception as e:
        print(f"Error al obtener la imagen: {e}")
        continue

# Cerrar todas las ventanas abiertas al finalizar
cv2.destroyAllWindows()
