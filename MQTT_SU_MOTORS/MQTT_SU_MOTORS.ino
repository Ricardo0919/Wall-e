#include <WiFi.h>
#include <PubSubClient.h>

// Ultrasonic sensor configuration
#define TRIG_PIN 2
#define ECHO_PIN 15

// H-bridge configuration for robot motors
#define IN1 23
#define IN2 22
#define IN3 16
#define IN4 4

#define FREQ 5000
#define PWM_CHANNEL1 0
#define PWM_CHANNEL2 1
#define RESOLUTION 8
int straightSpeed = 70;
int turnSpeed = 50;

// Buzzer
#define buzzer 18

// Claw motor configuration
#define CLAW_IN1 12  // D12
#define CLAW_IN2 13  // D13

#define CLAW_PWM_CHANNEL1 2
#define CLAW_PWM_CHANNEL2 3
int clawSpeed = 100;

// Wi-Fi configuration
const char* ssid = "Ricardo";
const char* password = "ricki1903#$";

// MQTT broker configuration
// Casa
//const char* mqttServer = "192.168.1.102";
// TEC
const char* mqttServer = "10.25.100.90";
const int mqttPort = 1883;

// Wi-Fi and MQTT clients
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

bool isMeasuring = false;  // Controls if the robot is moving
unsigned long lastMeasureTime = 0;
unsigned long lastMQTTPublishTime = 0;

void setup() {
  Serial.begin(115200);

  // Initialize pins
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(buzzer, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(CLAW_IN1, OUTPUT);
  pinMode(CLAW_IN2, OUTPUT);

  // Setup PWM channels for robot motors
  ledcSetup(PWM_CHANNEL1, FREQ, RESOLUTION);
  ledcSetup(PWM_CHANNEL2, FREQ, RESOLUTION);
  ledcAttachPin(IN1, PWM_CHANNEL1);
  ledcAttachPin(IN4, PWM_CHANNEL2);

  // Setup PWM channels for claw motors
  ledcSetup(CLAW_PWM_CHANNEL1, FREQ, RESOLUTION);
  ledcSetup(CLAW_PWM_CHANNEL2, FREQ, RESOLUTION);
  ledcAttachPin(CLAW_IN1, CLAW_PWM_CHANNEL1);
  ledcAttachPin(CLAW_IN2, CLAW_PWM_CHANNEL2);

  // Connect to Wi-Fi
  WiFi.begin(ssid, password);
  Serial.print("Connecting to Wi-Fi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nConnected to Wi-Fi");

  // Configure MQTT client
  mqttClient.setServer(mqttServer, mqttPort);
  mqttClient.setCallback(callback);
}

void loop() {
  // Reconnect if MQTT client is disconnected
  if (!mqttClient.connected()) {
    reconnect();
  }
  mqttClient.loop();

  if (isMeasuring) {
    measureAndAct();  // Execute measurement and actions only if active
  }
}

void forward() {
  ledcWrite(PWM_CHANNEL1, straightSpeed);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  ledcWrite(PWM_CHANNEL2, straightSpeed);
}

void stopMotors() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  ledcWrite(PWM_CHANNEL1, 0);
  ledcWrite(PWM_CHANNEL2, 0);
}

void turn() {
  ledcWrite(PWM_CHANNEL1, turnSpeed);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  ledcWrite(PWM_CHANNEL2, turnSpeed);

  unsigned long startTurn = millis();
  while (millis() - startTurn < 1000) {
    mqttClient.loop();  // Process MQTT messages while turning
  }

  stopMotors();
}

float measureDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH, 30000);
  if (duration > 0) {
    return (duration / 2.0) * 0.0343;
  }
  return -1;
}

void measureAndAct() {
  unsigned long currentTime = millis();
  if (currentTime - lastMeasureTime >= 100) {
    lastMeasureTime = currentTime;
    float distance = measureDistance();

    if (distance > 0 && distance <= 20.0) {
      Serial.println("Obstacle detected. Turning...");
      digitalWrite(buzzer, HIGH);
      delay(500);
      digitalWrite(buzzer, LOW);

      turn();
      forward();

      if (currentTime - lastMQTTPublishTime >= 1000) {
        lastMQTTPublishTime = currentTime;
        mqttClient.publish("esp32/distance", String(distance).c_str());
      }
    } else if (distance > 20.0) {
      Serial.println("Moving forward");
      forward();

      if (currentTime - lastMQTTPublishTime >= 1000) {
        lastMQTTPublishTime = currentTime;
        mqttClient.publish("esp32/distance", String(distance).c_str());
      }
    } else {
      Serial.println("Out of range");
      if (currentTime - lastMQTTPublishTime >= 1000) {
        lastMQTTPublishTime = currentTime;
        mqttClient.publish("esp32/distance", "Out of range");
      }
    }
  }
}

void openClaw() {
  Serial.println("Opening claw...");
  stopMotors();  // Stop the robot
  delay(300);    // Wait for 300 ms

  // Send 'Stop' to 'esp32/movement' topic
  mqttClient.publish("esp32/movement", "Stop");

  // Open the claw
  stopClaw();
  digitalWrite(CLAW_IN1, LOW);
  ledcWrite(CLAW_PWM_CHANNEL2, clawSpeed);
  delay(2100);  // Changed from 700 to 2100
  stopClaw();

  // Robot remains stopped after opening the claw
}

void closeClaw() {
  Serial.println("Closing claw...");
  // Close the claw without stopping the robot
  stopClaw();
  digitalWrite(CLAW_IN2, LOW);
  ledcWrite(CLAW_PWM_CHANNEL1, clawSpeed);
  delay(2100);  // Changed from 700 to 2100
  stopClaw();
}

void stopClaw() {
  ledcWrite(CLAW_PWM_CHANNEL1, 0);
  ledcWrite(CLAW_PWM_CHANNEL2, 0);
  digitalWrite(CLAW_IN1, LOW);
  digitalWrite(CLAW_IN2, LOW);
}
///////////////////////////////////////////////////////////////////////////////////
void foundObject() {
  int alignmentError = 0;
  int frameCenter = 160;  // Suponiendo un ancho de imagen de 320 píxeles (ajustar si es necesario)
  int objectCenter = 0;   // Este valor debe ser recibido como parámetro por MQTT
  int boundingBoxArea = 0;  // Simulación de un área del objeto (debería ser recibido o calculado)
  int approachThreshold = 500;  // Umbral para detenerse

  // Suponiendo que estos valores se ajusten dinámicamente con mensajes MQTT
  while (true) {
    // Ejemplo: lógica simulada
    alignmentError = frameCenter - objectCenter;  // Corregir orientación
    if (alignmentError > 10) {
      // Giro a la izquierda
      Serial.println("Corrigiendo orientación: girando a la izquierda");
      ledcWrite(PWM_CHANNEL1, 20);  // Reducir velocidad
      ledcWrite(PWM_CHANNEL2, 0);
    } else if (alignmentError < -10) {
      // Giro a la derecha
      Serial.println("Corrigiendo orientación: girando a la derecha");
      ledcWrite(PWM_CHANNEL1, 0);
      ledcWrite(PWM_CHANNEL2, 20);
    } else {
      // Movimiento recto
      Serial.println("Avanzando hacia el objeto");
      forward();
    }

    // Simulación de área de un bounding box (debería actualizarse dinámicamente)
    boundingBoxArea += 10;  // Aumentar progresivamente para simular acercamiento

    // Si el área supera el umbral, detenerse
    if (boundingBoxArea > approachThreshold) {
      Serial.println("Objeto alcanzado. Deteniéndose.");
      stopMotors();
      break;
    }

    delay(500);  // Pausa para simular tiempo de cálculo
  }
}

///////////////////////////////////////////////////////////////////////////////////////
void callback(char* topic, byte* payload, unsigned int length) {
  String message;
  for (unsigned int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  Serial.print("Received message on topic [");
  Serial.print(topic);
  Serial.print("]: ");
  Serial.println(message);

  if (String(topic) == "esp32/movement") {
    if (message == "Start") {
      Serial.println("Movement started");
      isMeasuring = true;  // Activate measurement and movement
    } else if (message == "Stop") {
      Serial.println("Movement stopped");
      isMeasuring = false;  // Stop measurement and movement
      stopMotors();

      // Publish messages indicating measurement and detection have stopped
      mqttClient.publish("esp32/distance", "Measurement has been stopped");
    }
  } else if (String(topic) == "esp32/claw") {
    if (message == "Open") {
      openClaw();
    } else if (message == "Close") {
      closeClaw();
    }
  } else if (String(topic) == "esp32/foundObject") {
    Serial.println("Tópico de acercamiento activado.");
    if (message == "Start") {
      foundObject();
    }
  }
}

void reconnect() {
  while (!mqttClient.connected()) {
    Serial.print("Attempting MQTT connection...");
    if (mqttClient.connect("ESP32Client")) {
      Serial.println("Connected");
      mqttClient.subscribe("esp32/movement");
      mqttClient.subscribe("esp32/claw");
      mqttClient.subscribe("esp32/foundObject"); 
      mqttClient.subscribe("esp32/foundObjectCenter");
      mqttClient.subscribe("esp32/foundObjectArea");

    } else {
      Serial.print("Failed, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" trying again in 5 seconds");
      delay(5000);
    }
  }
}
