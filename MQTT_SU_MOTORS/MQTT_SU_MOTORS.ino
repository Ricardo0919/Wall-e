#include <WiFi.h>
#include <PubSubClient.h>

// Configuración del sensor ultrasónico
#define TRIG_PIN 2    // Pin de Trig conectado a GPIO 2 del ESP32
#define ECHO_PIN 15   // Pin de Echo conectado a GPIO 15 del ESP32

// Configuración del puente H para los motores
#define IN1 23
#define IN2 22
#define IN3 21
#define IN4 19

// Configuración de la red Wi-Fi
//Casa
//const char* ssid = "INFINITUM4B9D_2.4";
//const char* password = "Wcmg4DjFH9";
//TEC
const char* ssid = "Ricardo";
const char* password = "ricki1903#$";

// Configuración del broker MQTT
//Casa
//const char* mqttServer = "192.168.1.85";
//TEC
const char* mqttServer = "10.25.99.111";
const int mqttPort = 1883;

// Instancias de cliente Wi-Fi y MQTT
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

// Función de callback para manejar mensajes MQTT
void callback(char* topic, byte* payload, unsigned int length) {
    // Convierte el payload a un String
    String message;
    for (int i = 0; i < length; i++) {
        message += (char)payload[i];
    }
    Serial.print("Mensaje recibido en el tópico [");
    Serial.print(topic);
    Serial.print("]: ");
    Serial.println(message);

    // Controlar el motor 1 según el mensaje recibido
    if (String(topic) == "esp32/movement") {
        if (message == "Forward") {
            //Motor1
            digitalWrite(IN1, HIGH);
            digitalWrite(IN2, LOW);
            //Motor2
            digitalWrite(IN3, HIGH);
            digitalWrite(IN4, LOW);
        } else if (message == "Backward") {
            //Motor1
            digitalWrite(IN1, LOW);
            digitalWrite(IN2, HIGH);
            //Motor2
            digitalWrite(IN3, LOW);
            digitalWrite(IN4, HIGH);
        } else if (message == "Stop") {
            //Motor1
            digitalWrite(IN1, LOW);
            digitalWrite(IN2, LOW);
            //Motor2
            digitalWrite(IN3, LOW);
            digitalWrite(IN4, LOW);
        }
    }
}

// Conectar al broker MQTT
void reconnect() {
    while (!mqttClient.connected()) {
        Serial.print("Intentando conectar al broker MQTT...");
        if (mqttClient.connect("ESP32Client")) {
            Serial.println("Conectado");
            // Suscribirse a los tópicos de los motores
            mqttClient.subscribe("esp32/movement");
        } else {
            Serial.print("Fallido, rc=");
            Serial.print(mqttClient.state());
            Serial.println(" Intentando de nuevo en 5 segundos");
            delay(5000);
        }
    }
}

void setup() {
    // Inicializar el puerto serie
    Serial.begin(115200);

    // Configurar el sensor ultrasónico
    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);

    // Configurar los pines de control de los motores
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);

    // Conectar a la red Wi-Fi
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("Conectado a Wi-Fi");

    // Configurar el servidor MQTT
    mqttClient.setServer(mqttServer, mqttPort);
    mqttClient.setCallback(callback);
}

void loop() {
    if (!mqttClient.connected()) {
        reconnect();
    }
    mqttClient.loop();

    // Medir la distancia con el sensor ultrasónico
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);

    long duration = pulseIn(ECHO_PIN, HIGH, 30000); // Timeout de 30ms

    if (duration > 0) {
        float distance = (duration / 2.0) * 0.0343;

        if (distance <= 20.0) {
          Serial.print("Distancia: ");
          Serial.print(distance);
          Serial.println(" cm");

          // Publicar la distancia al tópico MQTT
          String distanceStr = "Distancia: " + String(distance) + " cm";
          mqttClient.publish("esp32/distance", distanceStr.c_str());
        } 

        else {
          Serial.println("Out of range");
          mqttClient.publish("esp32/distance", "Out of range");
        }

    }

    delay(500);  // Esperar medio segundo antes de la próxima medición
}
