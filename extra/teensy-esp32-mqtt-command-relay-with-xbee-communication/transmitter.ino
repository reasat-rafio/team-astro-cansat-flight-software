#include <PubSubClient.h>
#include <SoftwareSerial.h>
#include <WiFi.h>

SoftwareSerial xbeeSerial(16, 17); // RX, TX - Replace with the pins connected to your XBee module
WiFiClient espClient;
PubSubClient mqttClient(espClient);

const int payloadSize = 256;
const char *ssid = "Orchie";
const char *password = "meyel1912";
const char *mqtt_server = "192.168.172.30";
const int mqtt_port = 1883; // Default MQTT port
const char *mqtt_topic = "ground_station/commands";

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
            mqttClient.subscribe("ground_station/commands"); // Subscribe to a topic
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

    char payloadString[length + 1]; // Add 1 for the null terminator
    for (int i = 0; i < length; i++) {
        payloadString[i] = (char)payload[i];
    }
    payloadString[length] = '\0'; // Null-terminate the string

    if (strcmp(topic, mqtt_topic) == 0) {
        Serial.println("ground_station Message is here !!!");
        if (length <= payloadSize) {
            xbeeSerial.print(payloadString);
            for (int i = length; i < payloadSize; i++) {
                xbeeSerial.print(" "); // Fill the remaining payload with spaces
            }
            Serial.print("Sent data to XBee: "); // Print the sent data
            Serial.println(payloadString);       // Print the sent data
        } else {
            Serial.println("Data exceeds payload size. Sending aborted.");
        }
    }
}
