#include <WiFi.h>
#include <PubSubClient.h>

// Motor control pins
#define IN3 12  // D12
#define IN4 13  // D13

// PWM settings
#define FREQ 5000
#define PWM_CHANNEL3 2
#define PWM_CHANNEL4 3
#define RESOLUTION 8
int clawSpeed = 100;

// Wi-Fi configuration
const char* ssid = "Ricardo";
const char* password = "ricki1903#$";

// MQTT broker configuration
const char* mqttServer = "192.168.1.102";
const int mqttPort = 1883;

// Wi-Fi and MQTT clients
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

void setup() {
  Serial.begin(115200);

  // Initialize motor control pins
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // Setup PWM channels
  ledcSetup(PWM_CHANNEL3, FREQ, RESOLUTION);
  ledcSetup(PWM_CHANNEL4, FREQ, RESOLUTION);

  // Attach PWM channels to motor control pins
  ledcAttachPin(IN3, PWM_CHANNEL3);
  ledcAttachPin(IN4, PWM_CHANNEL4);

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
}

void openClaw() {
  Serial.println("Opening claw...");
  stopClaw();
  digitalWrite(IN3, LOW);
  ledcWrite(PWM_CHANNEL4, clawSpeed);
  delay(500);
  ledcWrite(PWM_CHANNEL4, 0);
  stopClaw();
}

void closeClaw() {
  Serial.println("Closing claw...");
  stopClaw();
  digitalWrite(IN4, LOW);
  ledcWrite(PWM_CHANNEL3, clawSpeed);
  delay(500);
  ledcWrite(PWM_CHANNEL3, 0);
  stopClaw();
}

void stopClaw() {
  ledcWrite(PWM_CHANNEL3, 0);
  ledcWrite(PWM_CHANNEL4, 0);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

void callback(char* topic, byte* payload, unsigned int length) {
  String message;
  for (unsigned int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  Serial.print("Received message on topic [");
  Serial.print(topic);
  Serial.print("]: ");
  Serial.println(message);

  if (String(topic) == "esp32/claw") {
    if (message == "Open") {
      openClaw();
    } else if (message == "Close") {
      closeClaw();
    }
  }
}

void reconnect() {
  while (!mqttClient.connected()) {
    Serial.print("Attempting MQTT connection...");
    if (mqttClient.connect("ESP32Client")) {
      Serial.println("connected");
      mqttClient.subscribe("esp32/claw");
    } else {
      Serial.print("failed, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}
