#include <PubSubClient.h>
#include <SoftwareSerial.h>
#include <WiFi.h>

SoftwareSerial xbeeSerial(16, 17); // RX, TX - Replace with the pins connected to your XBee module
WiFiClient espClient;
PubSubClient mqttClient(espClient);

// const char *SSID = "Orchie";
// const char *PASSWORD = "meyel1912";
// const char *MQTT_SERVER = "192.168.136.30";
const char *SSID = "Orchie";
const char *PASSWORD = "meyel1912";
const char *MQTT_SERVER = "192.168.154.88";
const int MQTT_PORT = 1883; // Default MQTT port
const char *MQTT_TELEMETRY_TOPIC = "telemetry/data";
const char *MQTT_COMMANDS_TOPIC = "ground_station/commands";
const char *MQTT_COMMANDS_RESPONSE_TOPIC = "ground_station/commands_response";

void setup() {
    Serial.begin(9600);
    xbeeSerial.begin(9600); // Set the baud rate to match your XBee configuration
    setup_wifi();
    mqttClient.setServer(MQTT_SERVER, MQTT_PORT);
    mqttClient.setCallback(callback);
}

void loop() {
    if (!mqttClient.connected()) {
        reconnect();
    }
    mqttClient.loop();
    processReceivedXBeeData();
    processXbeeServoControl();
}

void processReceivedXBeeData() {
    static String receivedData = "";

    while (xbeeSerial.available()) {
        char character = xbeeSerial.read();
        receivedData += character;

        // Check if the received data contains both '<' and '>'
        if (receivedData.indexOf('<') != -1 && receivedData.indexOf('>') != -1) {
            int startIdx = receivedData.indexOf('<') + 1; // Get the index of '<'
            int endIdx = receivedData.indexOf('>');       // Get the index of '>'
            String extractedData = receivedData.substring(startIdx, endIdx);

            String modifiedExtractedData = extractedData.substring(1);

            // T means telemetry data
            if (extractedData.charAt(0) == 'T') {
                publishToMQTT(MQTT_TELEMETRY_TOPIC, modifiedExtractedData);

                // Print the received data
                Serial.println("Received data from XBee: " + modifiedExtractedData);
                receivedData = "";

            } else if (extractedData.charAt(0) == 'R') {
                publishToMQTT(MQTT_COMMANDS_RESPONSE_TOPIC, modifiedExtractedData);

                // Print the received data
                Serial.println("Received response data from XBee: " + modifiedExtractedData);

                // Clear the receivedData for the next payload
                receivedData = "";
            }
        }
    }
}

void processXbeeServoControl() {
    if (Serial.available()) {                             // Check if there is data available from Serial monitor
        String dataToSend = Serial.readStringUntil('\n'); // Read data from Serial monitor

        // Prepend 'S' to indicate servo control data
        String message = "<S" + dataToSend + ">";
        xbeeSerial.print(message);

        Serial.println("Sent Servo data to XBee: " + dataToSend); // Print the sent data
    }
}

void setup_wifi() {
    delay(10);
    Serial.println();
    Serial.print("Connecting to ");
    Serial.println(SSID);

    WiFi.begin(SSID, PASSWORD);

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
            mqttClient.subscribe("ground_station/commands");
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

    String payloadString = "";
    // char payloadString[length + 1];  // Add 1 for the null terminator
    for (int i = 0; i < length; i++) {
        payloadString += (char)payload[i];
    }

    if (strcmp(topic, MQTT_COMMANDS_TOPIC) == 0) {
        // Prepend 'C' to indicate command data
        String message = "<C" + payloadString + ">";
        xbeeSerial.print(message);
        Serial.print("ground_station Message is here !!!");
        Serial.println(message);

        Serial.print("Sent data to XBee: "); // Print the sent data
        payloadString = "";
    }
}

void publishToMQTT(const char *topic, String receivedData) {
    // Construct the message payload
    String payload = String(receivedData);

    if (mqttClient.publish(topic, payload.c_str())) {
        Serial.println("Publish successful");
    } else {
        Serial.println("Publish failed");
    }
}
