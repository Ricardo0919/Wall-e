import paho.mqtt.client as mqtt
import re  # Para trabajar con expresiones regulares y extraer la distancia


# Función que se ejecuta cuando te conectas al broker MQTT
def on_connect(client, userdata, flags, reasonCode, properties=None):
    print(f"Conectado con código de resultado {reasonCode}")
    client.subscribe("esp32/distance")  # Suscribirse al tópico esp32/distance


# Función que se ejecuta cuando se recibe un mensaje en un tópico suscrito
def on_message(client, userdata, msg):
    # Extraer el mensaje y decodificarlo
    message = msg.payload.decode()

    # Utilizar una expresión regular para buscar un número en el mensaje
    distance_match = re.search(r"[\d.]+", message)

    if distance_match:
        # Extraer el número de la distancia
        distance = float(distance_match.group())
        print(f"Distancia recibida: {distance} cm")

        # Comprobar si la distancia está en el rango entre 10 y 15 cm
        if 10 <= distance <= 15:
            client.publish("esp32/Turn", "Turn")
            print("Publicado 'Turn' en esp32/Turn")
        else:
            client.publish("esp32/Turn", "Nothing")
            print("Publicado 'Nothing' en esp32/Turn")
    else:
        print("Formato de mensaje de distancia no reconocido")


# Configuración del cliente MQTT
client = mqtt.Client()

# Asignación de funciones de callback
client.on_connect = on_connect
client.on_message = on_message

# Conexión al broker MQTT (puedes cambiar el host y puerto si es necesario)
client.connect("192.168.1.85", 1883, 60)

# Inicio de un hilo para manejar la red y las callbacks
client.loop_start()

# Bucle principal para evitar que el programa termine
try:
    while True:
        # Bucle inactivo, solo espera recibir mensajes
        pass

except KeyboardInterrupt:
    print("Desconectando...")

# Finaliza el hilo loop
client.loop_stop()
client.disconnect()
