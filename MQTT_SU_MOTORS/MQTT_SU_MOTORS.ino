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

// Buzzer
#define buzzer 18

// Configuración de la red Wi-Fi
//Casa
//const char* ssid = "INFINITUM4B9D_2.4";
//const char* password = "Wcmg4DjFH9";
//TEC
const char* ssid = "Ricardo";
const char* password = "ricki1903#$";

// Configuración del broker MQTT
// Casa
//const char* mqttServer = "192.168.1.85";
//TEC
const char* mqttServer = "10.25.109.41";
const int mqttPort = 1883;

// Instancias de cliente Wi-Fi y MQTT
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

bool isMeasuring = false;  // Variable para controlar si se está midiendo o no

void forward(){
  int motorSpeed = 100;

  //Motor1
  analogWrite(IN1, motorSpeed);
  analogWrite(IN2, 0);
  //Motor2
  analogWrite(IN3, 0);
  analogWrite(IN4, motorSpeed);
}

void turn() {
  int motorSpeed = 100;

  // Motor1 gira hacia adelante
  analogWrite(IN1, motorSpeed);
  analogWrite(IN2, 0);   // Apagar la entrada IN2 para que Motor1 gire hacia adelante

  // Motor2 gira hacia atrás
  analogWrite(IN3, motorSpeed);   // Apagar la entrada IN3 para que Motor2 gire hacia atrás
  analogWrite(IN4, 0);

  // Mantener el giro por 4 segundos
  delay(4000);

  // Detener los motores después de 4 segundos
  analogWrite(IN1, 0);   // Apagar Motor1
  analogWrite(IN2, 0);   // Apagar Motor1
  analogWrite(IN3, 0);   // Apagar Motor2
  analogWrite(IN4, 0);   // Apagar Motor2

  mqttClient.publish("esp32/movement", "Start");
}


// Función start para iniciar el movimiento y revisar el comando del tópico turn
void start() {

  // Activar medición
  isMeasuring = true;
  Serial.print("Start");
  
  forward();
}

void stop() {
  //Motor1
  analogWrite(IN1, LOW);
  analogWrite(IN2, LOW);
  //Motor2
  analogWrite(IN3, LOW);
  analogWrite(IN4, LOW);

  isMeasuring = false;  // Detener medición
  mqttClient.publish("esp32/distance", "Measurement has been stopped");
  mqttClient.publish("esp32/object", "Detection of objects has been stopped");
}

// Función para medir la distancia con el sensor ultrasónico
void measureDistance() {
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
      String distanceStr = String(distance);
      mqttClient.publish("esp32/distance", distanceStr.c_str());

      // Publicar la detección del objeto al tópico MQTT
      mqttClient.publish("esp32/object", "Object not found");

      // Si la distancia está entre 10 cm y 15 cm, hacer sonar el buzzer
      if (distance >= 10.0 && distance <= 15.0) {
        mqttClient.publish("esp32/object", "Object found");
        digitalWrite(buzzer, HIGH);  // Encender el buzzer
        delay(2000);  // Mantener el buzzer encendido durante 2 segundos
        digitalWrite(buzzer, LOW);   // Apagar el buzzer después de 2 segundos
      }

    } else {
      Serial.println("Out of range");
      mqttClient.publish("esp32/distance", "Out of range");
      mqttClient.publish("esp32/object", "Object not found");
    }
  }
}

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

  // Controlar el motor y la medición según el mensaje recibido
  if (String(topic) == "esp32/movement") {
    if (message == "Start") {
      start();  // Comenzar medición y mover motores
    } 
    else if (message == "Turn") {
      turn();   // Detener motores y medición
    }
    else if (message == "Stop") {
      stop();   // Detener motores y medición
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
            mqttClient.subscribe("esp32/turn");
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

    // Configurar el buzzer
    pinMode(buzzer, OUTPUT);  // Configurar el buzzer como salida

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

  // Si se está midiendo, continuar midiendo y publicando la distancia
  if (isMeasuring) {
    measureDistance();
  }

  delay(500);  // Esperar medio segundo antes de la próxima medición
}
