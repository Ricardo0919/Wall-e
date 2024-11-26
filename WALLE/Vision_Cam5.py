import cv2  # Asegúrate de importar el módulo correctamente

# Crear ventana y capturar video
cv2.namedWindow("Camara")
vc = cv2.VideoCapture(1)

if not vc.isOpened():
    print("Error: No se pudo abrir la cámara.")
    exit()

while True:
    next, frame = vc.read()
    if not next:
        print("Error: No se pudo capturar el frame.")
        break
    
    cv2.imshow("Camara", frame)
    if cv2.waitKey(50) >= 0:  # Salir al presionar cualquier tecla
        break

# Liberar recursos
vc.release()
cv2.destroyAllWindows()