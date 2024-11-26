import cv2

def main():
    url = "http://192.168.137.2:4747/video"  # Asegúrate de que este es el endpoint correcto
    cap = cv2.VideoCapture(url)

    if not cap.isOpened():
        print("No se pudo abrir el flujo de video.")
        return

    while True:
        ret, frame = cap.read()
        if not ret:
            print("No se pudo leer el fotograma.")
            break
        cv2.imshow('DroidCam', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

main()