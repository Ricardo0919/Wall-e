from flask import Flask, Response
import cv2

# Inicializa Flask
app = Flask(__name__)

# Variable global para almacenar el frame actual
current_frame = None

def update_frame(frame):
    """Actualiza el frame global con la imagen procesada."""
    global current_frame
    current_frame = frame

def generar_stream():
    """Genera el stream MJPEG a partir de los frames actuales."""
    global current_frame
    while True:
        if current_frame is None:
            continue  # Si no hay frame disponible, espera al siguiente ciclo

        # Codifica el frame en formato JPEG
        _, jpeg = cv2.imencode('.jpg', current_frame)
        frame = jpeg.tobytes()

        # Envía el frame como parte del stream MJPEG
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')

@app.route('/video')
def video():
    """Ruta para acceder al stream MJPEG."""
    return Response(generar_stream(), mimetype='multipart/x-mixed-replace; boundary=frame')

def iniciar_servidor():
    """Inicia el servidor Flask en un hilo separado."""
    from threading import Thread
    thread = Thread(target=lambda: app.run(host='0.0.0.0', port=5000, debug=False, use_reloader=False))
    thread.daemon = True
    thread.start()
