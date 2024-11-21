import network
import utime

SSID = "TuSSID"          # Reemplaza con tu nombre de red WiFi
PASSWORD = "TuPassword"  # Reemplaza con tu contraseña de red WiFi

def conectar_wifi():
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    if not wlan.isconnected():
        print("Conectando a la red WiFi...")
        wlan.connect(SSID, PASSWORD)
        
        # Esperar a que se conecte, máximo 10 segundos
        start_time = utime.time()
        while not wlan.isconnected() and (utime.time() - start_time) < 10:
            utime.sleep(1)
    
    if wlan.isconnected():
        print("Conexión exitosa!")
        print("Configuración de red:", wlan.ifconfig())
    else:
        print("Error de conexión.")

# Llamar a la función de conexión WiFi al iniciar
conectar_wifi()
