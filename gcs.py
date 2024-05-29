import paho.mqtt.client as mqtt
import serial
import time

# Configuration parameters
MQTT_SERVER = "localhost"
MQTT_PORT = 1883
MQTT_TELEMETRY_TOPIC = "telemetry/data"
MQTT_COMMANDS_TOPIC = "ground_station/commands"
MQTT_COMMANDS_RESPONSE_TOPIC = "ground_station/commands_response"
SERIAL_PORT = '/dev/ttyUSB0'  # Replace with your serial port
SERIAL_BAUD_RATE = 9600

# Initialize serial connection
ser = serial.Serial(SERIAL_PORT, SERIAL_BAUD_RATE)

# Callback when the client receives a CONNACK response from the server
def on_connect(client, userdata, flags, reason_code, properties):
    print(f"Connected with result code {reason_code}")
    # Subscribing in on_connect() means that if we lose the connection and
    # reconnect then subscriptions will be renewed.
    client.subscribe(MQTT_COMMANDS_TOPIC)

# Callback when a PUBLISH message is received from the server
def on_message(client, userdata, msg):
    print(f"Message arrived in topic: {msg.topic}")
    print(f"Message: {msg.payload.decode()}")

    if msg.topic == MQTT_COMMANDS_TOPIC:
        message = f"<C{msg.payload.decode()}>"
        ser.write(message.encode())
        print(f"Sent data to XBee: {message}")

# MQTT setup
client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
client.on_connect = on_connect
client.on_message = on_message
client.connect(MQTT_SERVER, MQTT_PORT, 60)

def publish_to_mqtt(topic, data):
    if client.publish(topic, data):
        print("Publish successful")
    else:
        print("Publish failed")

received_data = ""
def process_received_xbee_data():
    global received_data
    if ser.in_waiting > 0:
        character = ser.read().decode()
        received_data += character

        if '<' in received_data and '>' in received_data:
            start_idx = received_data.index('<') + 1
            end_idx = received_data.index('>')
            extracted_data = received_data[start_idx:end_idx]
            modified_extracted_data = extracted_data[1:]

            if extracted_data.startswith('T'):
                publish_to_mqtt(MQTT_TELEMETRY_TOPIC, modified_extracted_data)
                print(f"Received data from XBee: {modified_extracted_data}")
                received_data = ""
            elif extracted_data.startswith('R'):
                publish_to_mqtt(MQTT_COMMANDS_RESPONSE_TOPIC, modified_extracted_data)
                print(f"Received response data from XBee: {modified_extracted_data}")
                received_data = ""

def process_xbee_servo_control():
    if ser.in_waiting:
        data_to_send = input("Enter data for servo control: ")
        message = f"<S{data_to_send}>"
        ser.write(message.encode())
        print(f"Sent Servo data to XBee: {data_to_send}")

# Main loop
client.loop_start()

try:
    while True:
       process_received_xbee_data()
       # process_xbee_servo_control()
except KeyboardInterrupt:
    print("Exiting...")
    ser.close()
    client.loop_stop()
    client.disconnect()