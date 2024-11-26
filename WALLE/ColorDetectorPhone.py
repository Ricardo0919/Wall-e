import cv2
import numpy as np

# URL del stream de la cámara DroidCam
url = "http://192.168.137.2:4747/video"  # Cambia esta URL según sea necesario

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

def main():
    cap = cv2.VideoCapture(url)

    if not cap.isOpened():
        print("No se pudo abrir el flujo de video.")
        return

    while True:
        ret, frame = cap.read()
        if not ret:
            print("No se pudo leer el fotograma.")
            break

        # Convertir la imagen a espacio de color HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

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
        result = cv2.bitwise_and(frame, frame, mask=mask)

        # Mostrar la imagen original y la filtrada en la misma ventana
        combined = np.hstack((frame, result))
        cv2.imshow('Configuración HSV y Detección en Tiempo Real', combined)

        # Presiona 'q' para salir del bucle
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

main()
