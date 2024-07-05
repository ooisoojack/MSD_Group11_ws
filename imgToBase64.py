# Refer this website for more info on using paho MQTT client
# https://www.emqx.com/en/blog/how-to-use-mqtt-in-python

from paho.mqtt import client as mqtt_client
from PIL import Image
from io import BytesIO
import base64
import random
import logging
import time
import cv2
from pathlib import Path
import os

FIRST_RECONNECT_DELAY = 1
RECONNECT_RATE = 2
MAX_RECONNECT_COUNT = 12
MAX_RECONNECT_DELAY = 60

FRAME_WIDTH = 640
FRAME_HEIGHT = 480

cam_id = "/dev/v4l/by-id/usb-Sonix_Technology_Co.__Ltd._USB_2.0_Camera_SN0001-video-index"
broker = "localhost"
port = 1883
dustbin_ID = 1
img_topic = f"/group_11/{dustbin_ID}/img_frame"
take_pic_topic = "/group_11/take_picture"
client_id = f'python-mqtt-{random.randint(0, 1000)}'

class MQTTClientHandler:
    def __init__(self):
        super().__init__()
        self.takeAPicture = False
        self.gotPic = False
        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)


    def onConnect(self, client, userdata, flags, rc, properties = None):
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
        client = mqtt_client.Client(mqtt_client.CallbackAPIVersion.VERSION2, client_id)
        client.on_connect = self.onConnect
        client.on_disconnect = self.onDisconnect
        client.connect(broker, port)
        return client
    
    def on_message(self, client, userdata, msg):
        print(f"Received `{msg.payload.decode()}` from `{msg.topic}` topic")
        if msg.topic == '/group_11/take_picture':
            self.takeAPicture = msg.payload.decode()
            print(self.takeAPicture)

    def capturePicture(self):
        result, image = self.cap.read()
        #time.sleep(1)
        if self.takeAPicture:
            if result:
                self.gotPic = True
                myFile = Path(f"picTaken.jpg")
                if myFile.is_file():
                    try:
                        os.remove("picTaken.jpg")
                    except:
                        pass
                cv2.imwrite(f"picTaken.jpg", image)
            else:
                self.gotPic = False
                print("No image detected. Please try again")


    def imgToBase64(self):
        # Open the image file
        if self.gotPic:
            with open("picTaken.jpg", "rb") as f:
                buffer = BytesIO()
                image = Image.open(f)

                # resize the image so that the base64 code is not too long
                width, height = image.size
                new_size = (width // 2, height // 2)
                resized_image = image.resize(new_size)
                resized_image.save(buffer, format="JPEG")
                encoded_image = base64.b64encode(buffer.getvalue())
                print(str(encoded_image)[2:-1])
                return encoded_image
        
    def publishMQTT(self, topic, client):
        if self.takeAPicture:
            msg = self.imgToBase64()
        result = client.publish(topic, msg)
        status = result[0]

        if status == 0:
            print(f"Send `{msg}` to topic `{topic}`")
        else:
            print(f"Failed to send message to topic {topic}")

    def subscribeMQTT(self, topic, client: mqtt_client):
        client.subscribe(topic)
        client.on_message = self.on_message
    
    def reqAndRespHandler(self, client):
        self.capturePicture()
        self.subscribeMQTT(take_pic_topic, client)
        #print("lmao")
        if self.takeAPicture:
            self.publishMQTT(img_topic, client)
            self.takeAPicture = False

def main(args = None):
    clientClass = MQTTClientHandler()
    client = clientClass.connectMQTT()
    while True:
        client.loop_start()    
        clientClass.reqAndRespHandler(client)
        client.loop_stop() 

if __name__ == "__main__":
    main()
