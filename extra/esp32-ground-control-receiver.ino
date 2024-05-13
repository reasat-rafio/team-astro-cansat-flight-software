#include <PubSubClient.h>
#include <SoftwareSerial.h>
#include <WiFi.h>

SoftwareSerial xbeeSerial(16, 17); // RX, TX - Replace with the pins connected to your XBee module
WiFiClient espClient;
PubSubClient mqttClient(espClient);

const int payloadSize = 180;
const char *ssid = "Room-1010";
const char *password = "room1010";
const char *mqtt_server = "192.168.0.188";
const int mqtt_port = 1883; // Default MQTT port
const char *mqtt_telemetry_topic = "telemetry/data";

void setup() {
    Serial.begin(9600);
    xbeeSerial.begin(9600); // Set the baud rate to match your XBee configuration
    setup_wifi();
    mqttClient.setServer(mqtt_server, mqtt_port);
    mqttClient.setCallback(callback);
}

void loop() {
    if (!mqttClient.connected()) {
        reconnect();
    }
    mqttClient.loop();

    static String receivedData = "";

    while (xbeeSerial.available()) {
        char character = xbeeSerial.read();
        receivedData += character;
        if (receivedData.length() == payloadSize) { // Check if a complete payload is received
            publishToMQTT(receivedData);

            Serial.println("Received data from XBee: " + receivedData);
            receivedData = ""; // Reset receivedData for the next payload
        }
    }
}

void setup_wifi() {
    delay(10);
    Serial.println();
    Serial.print("Connecting to ");
    Serial.println(ssid);

    WiFi.begin(ssid, password);

    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }

    Serial.println("");
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
}

void reconnect() {
    while (!mqttClient.connected()) {
        Serial.print("Attempting MQTT connection...");
        if (mqttClient.connect("NodeMCUClient")) {
            Serial.println("connected");
            mqttClient.subscribe("topic"); // Subscribe to a topic
        } else {
            Serial.print("failed, rc=");
            Serial.print(mqttClient.state());
            Serial.println(" try again in 5 seconds");
            delay(5000);
        }
    }
}

void callback(char *topic, byte *payload, unsigned int length) {
    Serial.print("Message arrived in topic: ");
    Serial.println(topic);
    Serial.print("Message:");
    for (int i = 0; i < length; i++) {
        Serial.print((char)payload[i]);
    }
    Serial.println();
}

void publishToMQTT(String receivedData) {
    // Construct the message payload
    String payload = String(receivedData);

    if (mqttClient.publish(mqtt_telemetry_topic, payload.c_str())) {
        Serial.println("Publish successful");
    } else {
        Serial.println("Publish failed");
    }
}
