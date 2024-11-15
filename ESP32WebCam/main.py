import camera
import gc

def inicializar_camara():
    try:
        # Desinicializar si estaba previamente inicializada
        camera.deinit()
        
        # Configuración de pines y parámetros de la cámara
        camera.init(0, d0=4, d1=5, d2=18, d3=19, d4=36, d5=39, d6=34, d7=35,
                    format=camera.JPEG, framesize=camera.FRAME_VGA,
                    xclk_freq=camera.XCLK_20MHz, href=23, vsync=25, 
                    sioc=27, siod=26, xclk=21, pclk=22, fb_location=camera.PSRAM)
        
        camera.framesize(camera.FRAME_VGA)  # Resolución VGA
        camera.flip(0)                      # No voltear
        camera.mirror(0)                    # No espejear
        camera.brightness(0)                # Brillo neutro
        camera.contrast(0)                  # Contraste neutro
        camera.quality(10)                  # Alta calidad
        
        print("Cámara inicializada correctamente.")
    except Exception as e:
        print("Error al inicializar la cámara:", e)

# Capturar y guardar una imagen
def capturar_imagen():
    try:
        buf = camera.capture()
        with open("/captura.jpg", "wb") as f:
            f.write(buf)
        print("Imagen capturada y guardada como 'captura.jpg'.")
        del buf
        gc.collect()
    except Exception as e:
        print("Error al capturar la imagen:", e)

# Inicializar la cámara y capturar una imagen al arrancar
inicializar_camara()
capturar_imagen()
