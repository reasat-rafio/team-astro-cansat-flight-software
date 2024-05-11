#include <PubSubClient.h>
#include <SoftwareSerial.h>
#include <WiFi.h>

SoftwareSerial xbeeSerial(16, 17); // RX, TX - Replace with the pins connected to your XBee module
WiFiClient espClient;
PubSubClient client(espClient);

const int payloadSize = 256;
const char *ssid = "Orchie";
const char *password = "meyel1912";
const char *mqtt_server = "192.168.172.30";
const int mqtt_port = 1883; // Default MQTT port
const char *mqtt_topic = "telemetry/data";

void setup() {
    Serial.begin(9600);
    xbeeSerial.begin(9600); // Set the baud rate to match your XBee configuration
    setup_wifi();
    client.setServer(mqtt_server, mqtt_port);
    client.setCallback(callback);
}

void loop() {
    if (!client.connected()) {
        reconnect();
    }
    client.loop();

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
    while (!client.connected()) {
        Serial.print("Attempting MQTT connection...");
        if (client.connect("NodeMCUClient")) {
            Serial.println("connected");
            client.subscribe("topic"); // Subscribe to a topic
        } else {
            Serial.print("failed, rc=");
            Serial.print(client.state());
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
    // Extract values from received data
    int accelerationX = getValue(receivedData, "Acceleration X:");
    int accelerationY = getValue(receivedData, "Acceleration Y:");
    int gyroscopeZ = getValue(receivedData, "Gyroscope Z:");
    float latitude = getValueFloat(receivedData, "Latitude:");
    float longitude = getValueFloat(receivedData, "Longitude:");
    float altitude = getValueFloat(receivedData, "Altitude:");
    // int distance = getValue(receivedData, "Distance:");
    float temperature = getValueFloat(receivedData, "Temperature:");
    float pressure = getValueFloat(receivedData, "Pressure:");
    float voltage = getValueFloat(receivedData, "Voltage:");

    // Construct the message payload
    String payload = String("2043,null,null,null,null,") + String(altitude) + ",null,null,null," + String(temperature) + "," + String(voltage) + "," + String(pressure) + ",null," + String(latitude) + "," + String(longitude) + ",null," + String(accelerationX) + "," + String(accelerationY) + "," + String(gyroscopeZ) + ",null,null";

    if (client.publish(mqtt_topic, payload.c_str())) {
        Serial.println("Publish successful");
    } else {
        Serial.println("Publish failed");
    }
}

int getValue(String data, String key) {
    int index = data.indexOf(key);
    if (index == -1) {
        return 0; // Return 0 if key not found
    }
    return data.substring(index + key.length()).toInt();
}

float getValueFloat(String data, String key) {
    int index = data.indexOf(key);
    if (index == -1) {
        return 0.0; // Return 0.0 if key not found
    }
    return data.substring(index + key.length()).toFloat();
}
