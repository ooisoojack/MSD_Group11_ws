# Refer this website for more info on using paho MQTT client
# https://www.emqx.com/en/blog/how-to-use-mqtt-in-python

# Refer this website on how to use GPIOD library to use Raspberry Pi GPIOs
# https://www.tomshardware.com/how-to/control-raspberry-pi-5-gpio-with-python-3


# import necessary modules
import os
import serial
from serial import SerialException
import gpiod
from paho.mqtt import client as mqtt_client
from PIL import Image
from io import BytesIO
import base64
import random
import logging
import time
import cv2
from pathlib import Path
import argparse
from ultralytics import YOLO
import supervision as sv
import numpy as np


# MQTT parameters
FIRST_RECONNECT_DELAY = 1
RECONNECT_RATE = 2
MAX_RECONNECT_COUNT = 12
MAX_RECONNECT_DELAY = 60
broker = "localhost"
port = 1883
client_id = f'python-mqtt-{random.randint(0, 1000)}'

# serial comms parameters
serial_id = "/dev/serial/by-id/"

# camera parameters
FRAME_WIDTH = 640
FRAME_HEIGHT = 480
cam_id = "/dev/v4l/by-id/usb-Sonix_Technology_Co.__Ltd._USB_2.0_Camera_SN0001-video-index"

# GPIO
relayPin = 17   # GPIO 17

# dustbin parameters and topics
dustbin_ID = 1
img_topic = f"/group_11/{dustbin_ID}/img_frame"
take_pic_topic = "/group_11/take_picture"
sensor_topic = f"/group_11/{dustbin_ID}/sensor_data"
waste_topic = f"/group_11/{dustbin_ID}/waste_details"


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="YOLOv8 Live")
    parser.add_argument(
        "--webcam-resolution",
        default=[1280, 720],
        nargs=2,
        type=int
    )
    args = parser.parse_args()
    return args


class serialHandler():
    # initialize the class and the serial port
    def __init__(self):
        super().__init__()
        self.trashName = ""
        self.whichBinPartition = 0

        while True:
            try:
                self.arduinoPort = serial.Serial(f'/dev/serial/by-id/{serial_id}', 115200)
                break
            except SerialException as serr:
                if not self.errMsgOnce:
                    self.errMsgOnce = True
                    print(serr)
                    time.sleep(1)
                    self.errMsgOnce = False

    # attempt to reopen the ESP32 port if a serial exception is raised
    def reopenPort(self):
        try:
            self.arduinoPort = serial.Serial(f'/dev/serial/by-path/{serial_id}', 115200)
            self.get_logger().info("Serial connection reestablished")
 
        # if the port failed to be opened
        except SerialException or OSError:
            if not self.errMsgOnce:
                self.errMsgOnce = True
                print("Error talking to Main Controller! Please check the cable.")
                time.sleep(1)
                self.errMsgOnce = False

    def serialReceive(self):

        try:
            incomingSerial = self.arduinoPort.readline()
            dataToString = str(incomingSerial)
            splitData = dataToString[2:-5].split("/")

        except SerialException or OSError:
            self.disconnected = True
            self.reopenPort()


    def serialTransmit(self):

        try:
            self.arduinoPort.write(f'{self.whichBinPartition}\r'.encode())
            self.disconnected = False
            #self.get_logger().info(f"[Serial Transmit] x: {self.x}, y: {self.y}, w: {self.w}, intake: {self.ballIntake}, eject: {self.ballEjector}")

        except SerialException:
            self.disconnected = True
            self.reopenPort()
            #self.get_logger().error("ESP32 connection lost! Try replugging the USB cable and restart...")


class cameraHandler():
    # initialize the class and the camera
    def __init__(self):
        super().__init__()
        self.takeAPicture = False
        self.prevDetectTime = 0
        self.gotPic = False
        self.result = None
        self.image = None
        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)

        self.model = YOLO("yolov8n.pt")


    # class method to handle camera taking photos
    def capturePicture(self):
        self.result, self.image = self.cap.read()
        if self.takeAPicture:
            # if there is image data, save it as picTaken.JPG in the current directory
            if self.result:
                self.gotPic = True
                myFile = Path(f"picTaken.jpg")
                if myFile.is_file():
                    try:
                        os.remove("picTaken.jpg")
                    except:
                        pass
                cv2.imwrite(f"picTaken.jpg", self.image)

            # otherwise, print an error message
            else:
                self.gotPic = False
                print("No image detected. Please try again")

    # since Raspberry Pi 5 is still not a good computer to do live detection, only 
    def detectTrashFromImage(self):
        # detect the objects in the color images
        results = self.model.predict(source = color_image1, boxes = False, verbose = False, show = False, conf = 0.20, max_det = 3)[0]
        names = self.model.names
        detections = sv.Detections.from_ultralytics(results)





# GPIOHandler class to control any GPIO pins on the Raspberry Pi 5
# Currently, it is used to control a white LED strip only through a relay 
class GPIOHandler:
    # initialize the class and the GPIO
    def __init__(self):
        super().__init__()
        self.takeAPicture = False
        self.chip = gpiod.Chip('gpiochip4') 
        self.relayLine = self.chip.get_line(relayPin)
        self.relayLine.request(consumer="LED", type=gpiod.LINE_REQ_DIR_OUT)
    
    # class method to control the LED strip
    def controlLEDStrip(self):
        try:
            if self.takeAPicture:
                self.relayLine.set_value(1) # turn on the LED
            else:
                self.relayLine.set_value(0) # turn off the LED
        except Exception:
            self.relayLine.release()


# MQTTClientHandler class to take care of all MQTT-related data transmissions
class MQTTClientHandler:
    # initialize the class and the camera
    def __init__(self):
        super().__init__()
        self.takeAPicture = False
        self.gotPic = False

    # handles connection to the Mosquitto MQTT
    def onConnect(self, client, userdata, flags, rc, properties = None):
        if rc == 0:
            print("Connected to Mosquitto MQTT Broker")
        else:
            print("Failed to connect to Mosquitto MQTT broker, return code %d\n", rc)

    # handles disconnection from Mosquitto MQTT
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

    # class method to establish MQTT connection and take care of disconnections 
    def connectMQTT(self):
        client = mqtt_client.Client(mqtt_client.CallbackAPIVersion.VERSION2, client_id)
        client.on_connect = self.onConnect
        client.on_disconnect = self.onDisconnect
        client.connect(broker, port)
        return client
    
    # class method to handle MQTT data coming from NodeRED or other sources
    def on_message(self, client, userdata, msg):
        print(f"Received `{msg.payload.decode()}` from `{msg.topic}` topic")

        # if we received a request to take a photo
        if msg.topic == '/group_11/take_picture':
            self.takeAPicture = msg.payload.decode()

    # class method to handle conversion of RGB frame to base64 format
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
    
    # class method to handle message publishing task
    def publishMQTT(self, topic, client):
        if self.takeAPicture:
            msg = self.imgToBase64()
        result = client.publish(topic, msg)
        status = result[0]

        if status == 0:
            print(f"Send `{msg}` to topic `{topic}`")
        else:
            print(f"Failed to send message to topic {topic}")

    # class method to handle message subscribing task
    def subscribeMQTT(self, topic, client: mqtt_client):
        client.subscribe(topic)
        client.on_message = self.on_message

    # MAIN class method to execute all MQTT tasks
    def reqAndRespHandler(self, client):
        self.capturePicture()
        self.subscribeMQTT(take_pic_topic, client)
        if self.takeAPicture:
            self.publishMQTT(img_topic, client)
            self.takeAPicture = False

def main(args = None):
    client_class = MQTTClientHandler()
    client = client_class.connectMQTT()

    camera_handler = cameraHandler()
    gpio_handler = GPIOHandler()

    while True:
        gpio_handler.controlLEDStrip()
        client.loop_start()    
        client_class.reqAndRespHandler(client)
        client.loop_stop() 

if __name__ == "__main__":
    main()
