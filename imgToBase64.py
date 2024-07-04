from paho.mqtt import client as mqtt_client
from PIL import Image
from io import BytesIO
import base64
import random
import logging
import time

FIRST_RECONNECT_DELAY = 1
RECONNECT_RATE = 2
MAX_RECONNECT_COUNT = 12
MAX_RECONNECT_DELAY = 60

broker = "localhost"
port = 1883
dustbin_ID = 1
topic = f"/group_11/{dustbin_ID}/img_frame"
client_id = f'python-mqtt-{random.randint(0, 1000)}'

class MQTTClientHandler:
    def __init__(self):
        super().__init__()

    def onConnect(self, client, userdata, flags, rc, properties):
        if rc == 0:
            print("Connected to Mosquitto MQTT Broker")
        else:
            print("Failed to connect to Mosquitto MQTT broker, return code %d\n", rc)

    def onDisconnect(self, client, userdata, rc):
        logging.info("Disconnected from Mosquitto MQTT with result code: %s", rc)
        reconnectCount, reconnectDelay = 0, FIRST_RECONNECT_DELAY
        while reconnectCount < MAX_RECONNECT_COUNT:
            logging.info("Reconnecting in %d seconds...", reconnectDelay)
            time.sleep(reconnectDelay)

            try:
                client.reconnect()
                logging.info("Reconnected successfully!")
                return
            except Exception as err:
                logging.error("%s. Reconnect failed. Retrying...", err)

            reconnectDelay *= RECONNECT_RATE
            reconnectDelay = min(reconnectDelay, MAX_RECONNECT_DELAY)
            reconnectCount += 1
        logging.info("Reconnect failed after %s attempts. Exiting...", reconnectCount)


    def connectMQTT(self):
        client = mqtt_client.Client(client_id)

        client.on_connect = self.onConnect
        client.on_disconnect = self.onDisconnect
        client.connect(broker, port)
        return client
    
    def on_message(client, userdata, msg):
        print(f"Received `{msg.payload.decode()}` from `{msg.topic}` topic")


    def publishMQTT(self, client):
        msgCount = 1
        while True:
            time.sleep(1)
            msg = f"messages: {msgCount}"
            result = client.publish(topic, msg)
            status = result[0]

            if status == 0:
                print(f"Send `{msg}` to topic `{topic}`")
            else:
                print(f"Failed to send message to topic {topic}")
            msgCount += 1
            if msgCount > 5:
                break

    def subscribe(self, client: mqtt_client):
        client.subscribe(topic)
        client.on_message = self.on_message

def imgToBase64():
    # Open the image file
    with open("frc.jpeg", "rb") as f:
        buffer = BytesIO()
        image = Image.open(f)

        # resize the image so that the base64 code is not too long
        width, height = image.size
        new_size = (width // 2, height // 2)
        resized_image = image.resize(new_size)
        resized_image.save(buffer, format="JPEG")
        encoded_image = base64.b64encode(buffer.getvalue())
        #print(str(encoded_image)[2:-1])

def main(args = None):
    while True:
        clientClass = MQTTClientHandler()
        client = clientClass.connectMQTT()



if __name__ == "__main__":
    main()
