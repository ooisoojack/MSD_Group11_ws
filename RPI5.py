# Refer this website for more info on using paho MQTT client
# https://www.emqx.com/en/blog/how-to-use-mqtt-in-python

# Refer this website on how to use GPIOD library to use Raspberry Pi GPIOs
# https://www.tomshardware.com/how-to/control-raspberry-pi-5-gpio-with-python-3


# TODO
# 1) Write another topic to save and publish the processed image (with bounding boxes and labels)
# 2) Write parsing code to parse the incoming serial data and send them all to respective topics
# 3) allow the whole process to know whether it should stop or not
# 4) nodered side please think about UI, like add buttons to manually call cleaner, show what dustbins are available
# 5) test the gpio code to turn on or off the relay for the LED

# import necessary modules
import os
import json
import subprocess
import psutil
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

# ---------------------------
# ---------------------------
#
# GLOBAL VARIABLES
# 
# ---------------------------
# ---------------------------

# MQTT parameters
FIRST_RECONNECT_DELAY = 1
RECONNECT_RATE = 2
MAX_RECONNECT_COUNT = 12
MAX_RECONNECT_DELAY = 60
# broker = "0.tcp.ap.ngrok.io"  # For different network application
# port = 18518
broker = "localhost"    # For local network application
port = 1883
client_id = f'python-mqtt-{random.randint(0, 1000)}'

# serial comms parameters
serial_id = "/dev/serial/by-id/"

# camera parameters
FRAME_WIDTH = 640
FRAME_HEIGHT = 480
cam_id = "/dev/v4l/by-id/usb-Sonix_Technology_Co.__Ltd._USB_2.0_Camera_SN0001-video-index"
AUTOTAKE_PHOTO_DELAY = 10000    # 1 minute delay
#create a rectangle at left side of screen
ZONE_POLYGON = np.array([
    [0.1,0.7],
    [0.9,0.7],
    [0.9, 1],
    [0.1, 1]
])

# GPIO
relayPin = 17   # GPIO 17

# dustbin parameters and topics
dustbin_ID = 1
img_topic = f"/group_11/{dustbin_ID}/img_frame"
take_pic_topic = "/group_11/take_picture"
sensor_topic = f"/group_11/{dustbin_ID}/sensor_data"
metal_waste_topic = f"/group_11/{dustbin_ID}/metal_waste"
battery_waste_topic = f"/group_11/{dustbin_ID}/battery_waste"
electronic_waste_topic = f"/group_11/{dustbin_ID}/electronic_waste"
general_dry_waste_topic = f"/group_11/{dustbin_ID}/general_dry_waste"
general_wet_waste_topic = f"/group_11/{dustbin_ID}/general_wet_waste"

rssi_topic = f"/group_11/{dustbin_ID}/rssi"
dustbin_ID_topic = f"/group_11/{dustbin_ID}/id"
cpu_usage_topic = f"/group_11/{dustbin_ID}/cpu_usage"
ram_usage_topic = f"/group_11/{dustbin_ID}/ram_usage"
cpu_temp_topic = f"/group_11/{dustbin_ID}/cpu_temp"


got_metal_waste = 0
got_battery_waste = 0
got_electronic_waste = 0
got_general_dry_waste = 0
got_general_wet_waste = 0


# logics
gotPic = False
autoTakeAPic = False
manualTakeAPic = False

# image data
detectedTrash = []

# -------------------------
# -------------------------
#
# SERIAL HANDLER CLASS
#
# -------------------------
# -------------------------

class serialHandler():
    # initialize the class and the serial port
    def __init__(self):
        super().__init__()
        self.trashName = ""
        self.whichBinPartition = 0
        self.objectDetected = False
        self.continueOp = False

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

    # ------------------------------
    # SERIAL PORT HANDLING FUNCTION
    # ------------------------------

    # attempt to reopen the ESP32 port if a serial exception is raised
    def reopenPort(self):
        try:
            self.arduinoPort = serial.Serial(f'/dev/serial/by-path/{serial_id}', 115200)
            print("Serial connection reestablished")
 
        # if the port failed to be opened
        except SerialException or OSError:
            if not self.errMsgOnce:
                self.errMsgOnce = True
                print("Error talking to Main Controller! Please check the cable.")
                time.sleep(1)
                self.errMsgOnce = False

    # ------------------------------
    # SERIAL DATA PARSING FUNCTIONS
    # ------------------------------

    def serialReceive(self):
        try:
            incomingSerial = self.arduinoPort.readline()
            dataToString = str(incomingSerial)
            splitData = dataToString[2:-5].split("/")

            if int(splitData[0]) == 0:
                self.objectDetected = False
            else:
                self.objectDetected = True

            if int(splitData[1]) == 0:
                self.continueOp = False
            else:
                self.continueOp = True

        except SerialException or OSError:
            self.disconnected = True
            self.reopenPort()


    def serialTransmit(self):
        global autoTakeAPic, gotPic, detectedTrash
        global got_general_dry_waste, got_general_wet_waste, got_battery_waste, got_electronic_waste, got_metal_waste

        if autoTakeAPic and gotPic:
            if detectedTrash[0] == "Metal":
                self.whichBinPartition = 0
                got_metal_waste = 1
                got_battery_waste = 0
                got_electronic_waste = 0
                got_general_wet_waste = 0
                got_general_dry_waste = 0

            elif detectedTrash[0] == "Battery":
                self.whichBinPartition = 1
                got_metal_waste = 0
                got_battery_waste = 1
                got_electronic_waste = 0
                got_general_wet_waste = 0
                got_general_dry_waste = 0

            elif detectedTrash[0] == "Electronic Waste":
                self.whichBinPartition = 2
                got_metal_waste = 0
                got_battery_waste = 0
                got_electronic_waste = 1
                got_general_wet_waste = 0
                got_general_dry_waste = 0

            elif detectedTrash[0] == "General Dry Waste":
                self.whichBinPartition = 3
                got_metal_waste = 0
                got_battery_waste = 0
                got_electronic_waste = 0
                got_general_wet_waste = 0
                got_general_dry_waste = 1


            elif detectedTrash[0] == "General Wet Waste":
                self.whichBinPartition = 4
                got_metal_waste = 0
                got_battery_waste = 0
                got_electronic_waste = 0
                got_general_wet_waste = 1
                got_general_dry_waste = 0
            
            else:
                got_metal_waste = 0
                got_battery_waste = 0
                got_electronic_waste = 0
                got_general_wet_waste = 0
                got_general_dry_waste = 0

            try:
                self.arduinoPort.write(f'{self.whichBinPartition}\r'.encode())
                self.disconnected = False

            except SerialException:
                self.disconnected = True
                self.reopenPort()

    # -------------
    # TASK FUNCTION
    # -------------

    def mainFunction(self):
        self.serialReceive()
        self.serialTransmit()





# -------------------------
# -------------------------
#
# CAMERA HANDLER CLASS
#
# -------------------------
# -------------------------

class cameraHandler():
    # initialize the class and the camera
    def __init__(self):
        super().__init__()
        
        self.prevDetectTime = 0
        self.prevInterruptTime = self.currentMillis()

        self.result = None
        self.image = None
        self.processedImage = None
        self.doneDetection = False
                
        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)

        self.model = YOLO("yolov8n.pt")

        #Supervision Tracking function
        self.tracker = sv.ByteTrack()

        #Color for each class for tracking and label
        self.colors = sv.ColorPalette.from_hex(['#0000ff',	'#5D3FD3','#ff0000'])

        #Draw box on tracking object
        self.box_annotator = sv.BoxCornerAnnotator(color= self.colors,thickness=5,corner_length = 15)

        #label info on the traking object
        self.label_annotator = sv.LabelAnnotator(color=self.colors)

        #create instance of polygon for zone
        # self.zone_polygon = (ZONE_POLYGON * np.array((FRAME_WIDTH, FRAME_HEIGHT))).astype(int)
        # self.zone = sv.PolygonZone(polygon = self.zone_polygon, frame_resolution_wh= (FRAME_WIDTH, FRAME_HEIGHT))
        # self.zone_annotator = sv.PolygonZoneAnnotator(zone=self.zone, color=sv.Color.green())


    def currentMillis(self):
        return round(time.time() * 1000)

    # class method to handle camera taking photos
    def capturePicture(self):
        global gotPic, manualTakeAPic, autoTakeAPic

        self.result, self.image = self.cap.read()

        # if we received a request to take a picture
        if manualTakeAPic:
            # if there is image data, save it as manualPicTaken.JPG in the current directory
            if self.result:
                myFile = Path(f"manualPicTaken.jpg")
                if myFile.is_file():
                    try:
                        os.remove("manualPicTaken.jpg")
                    except:
                        pass
                cv2.imwrite(f"manualPicTaken.jpg", self.image)
                gotPic = True

            # otherwise, print an error message
            else:
                gotPic = False
                print("No manual image detected. Please try again")

        # if we received a request to take a picture
        elif autoTakeAPic:

            # if there is image data, save it as autoPicTaken.JPG in the current directory
            if self.doneDetection:
                myFile = Path(f"autoPicTaken.jpg")
                if myFile.is_file():
                    try:
                        os.remove("autoPicTaken.jpg")
                    except:
                        pass
                cv2.imwrite(f"autoPicTaken.jpg", self.processedImage)
                gotPic = True
                self.doneDetection = False

            # otherwise, print an error message
            else:
                gotPic = False
                print("No auto image detected. Please try again")


    # since Raspberry Pi 5 is still not a good computer to do live detection, only image source will be used to do detection
    def detectTrashFromImage(self):
        global autoTakeAPic, detectedTrash
        # detect the objects in the color images
        if autoTakeAPic:
            print("I got a picture!, processing it now")
            self.detectedTrash = []
            results = self.model.predict(source = self.image, show_boxes = False, verbose = False, show = False, conf = 0.20, max_det = 3)[0]
            names = self.model.names
            detections = sv.Detections.from_ultralytics(results)
            detections = self.tracker.update_with_detections(detections)

            # display the details of the detections at the top of each object's bounding boxes
            labels = [
                f"{self.model.model.names[class_id]} {confidence:0.2f} {results.boxes.xyxy[0][0]}"
                for _, _, confidence, class_id, _
                in detections
            ]
            
            # obtain the results of the detections
            for r in results:
                for c in r.boxes.cls:
                    detectedTrash.append(names[int(c)]) # append the trash detected into the array
            
            #Draw the box on screen
            self.processedImage = self.box_annotator.annotate(scene=self.image, detections=detections)

            #Draw the labels on screen
            self.processedImage = self.label_annotator.annotate(
                scene=self.processedImage, detections=detections, labels=labels)

            self.doneDetection = True
            self.prevInterruptTime = self.currentMillis()   # reset the previous interrupt time


    def mainFunction(self):
        global gotPic, autoTakeAPic

        # if there is no trash for quite some time already, automatically take a photo every time the set period of  
        if self.currentMillis() - self.prevInterruptTime >= AUTOTAKE_PHOTO_DELAY:
            print("Timeout triggered, taking a photo with detections...")
            autoTakeAPic = True
            self.prevInterruptTime = self.currentMillis()
        self.capturePicture()
        self.detectTrashFromImage()



# -------------------------
# -------------------------
#
# GPIO HANDLER CLASS
#
# -------------------------
# -------------------------

class GPIOHandler:
    # initialize the class and the GPIO
    def __init__(self):
        super().__init__()
        self.takeAPicture = False
        self.chip = gpiod.Chip('gpiochip4') 
        self.relayLine = self.chip.get_line(relayPin)
        self.relayLine.request(consumer="LED", type=gpiod.LINE_REQ_DIR_OUT)
    
    # --------------------
    # TASK FUNCTION
    # --------------------

    # class method to control the LED strip
    def controlLEDStrip(self):
        global manualTakeAPic, autoTakeAPic
        try:
            if autoTakeAPic or manualTakeAPic:
                self.relayLine.set_value(1) # turn on the LED
            else:
                self.relayLine.set_value(0) # turn off the LED
        except Exception:
            self.relayLine.release()

    def mainFunction(self):
        self.controlLEDStrip()


# -------------------------
# -------------------------
#
# MQTT CLIENT HANDLER CLASS
#
# -------------------------
# -------------------------

# MQTTClientHandler class to take care of all MQTT-related data transmissions
class MQTTClientHandler:
    # initialize the class and the camera
    def __init__(self):
        super().__init__()

        self.doneConversion = False
        self.msgToPub = None
    # ------------------------------------
    # MQTT CONNECTION HANDLING FUNCTIONS
    # ------------------------------------

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
    

    # ------------------------------------
    # MQTT MESSAGE HANDLING FUNCTIONS
    # ------------------------------------

    # class method to handle MQTT data coming from NodeRED or other sources
    def on_message(self, client, userdata, msg):
        global takeAPicture, manualTakeAPicture
        print(f"Received `{msg.payload.decode()}` from `{msg.topic}` topic")

        # if we received a request to take a photo
        if msg.topic == '/group_11/take_picture':
            print("Calling Camera Handler to take photo...")
            manualTakeAPicture = msg.payload.decode()

    # class method to handle message publishing task
    def publishMQTT(self, topic, client):
        global autoTakeAPic, manualTakeAPic, gotPic

        # if we received a request to take a picture, receive the base64 code of the picture
        if topic == img_topic and gotPic:
            self.msgToPub = self.imgToBase64()
            self.doneConversion = True
        
        elif topic == rssi_topic:
            self.msgToPub = self.getRSSI("wlo1")

        elif topic ==  metal_waste_topic:
            self.msgToPub = got_metal_waste

        elif topic ==  battery_waste_topic:
            self.msgToPub = got_battery_waste

        elif topic ==  electronic_waste_topic:
            self.msgToPub = got_electronic_waste
        
        elif topic ==  general_dry_waste_topic:
            self.msgToPub = got_general_dry_waste

        elif topic ==  general_wet_waste_topic:
            self.msgToPub = got_general_wet_waste

        elif topic ==  cpu_usage_topic:
            self.msgToPub = self.getCPUUsage()
        
        elif topic ==  ram_usage_topic:
            self.msgToPub = self.getRAMUsage()

        elif topic ==  cpu_temp_topic:
            self.msgToPub = self.getCPUTemp()

        if self.msgToPub != None:
            result = client.publish(topic, self.msgToPub)
            status = result[0]

            if status == 0:
                print(f"Send `{self.msgToPub}` to topic `{topic}`")
            else:
                print(f"Failed to send message to topic {topic}")
        
        # reset the message
        self.msgToPub = None

    # class method to handle message subscribing task
    def subscribeMQTT(self, topic, client: mqtt_client):
        client.subscribe(topic)
        client.on_message = self.on_message


    # ------------------------------------
    # TASK FUNCTIONS
    # ------------------------------------

    def getCPUUsage(self):
        # Get CPU usage percentage
        return psutil.cpu_percent(interval=1)  # Interval of 1 second

    def getRAMUsage(self):
        # Get RAM usage information
        ram_info = psutil.virtual_memory()
        percent_used = ram_info.percent
        return percent_used

    def getCPUTemp(self):
        return psutil.sensors_temperatures()['coretemp'][0].current

    # Get the signal strength of the connection
    def getRSSI(self, interface):
        try:
            # Run the iwconfig command
            result = subprocess.run(['iwconfig', interface], capture_output=True, text=True)
            output = result.stdout

            # Find the line that contains 'Signal level'
            for line in output.split('\n'):
                if 'Signal level' in line:
                    # Extract the signal level
                    parts = line.split('Signal level=')
                    if len(parts) > 1:
                        signal_level = parts[1].split()[0]
                        return signal_level
            return None
        except Exception as e:
            print(f"An error occurred: {e}")
            return None

    # class method to handle conversion of RGB frame to base64 format
    def imgToBase64(self):
        global gotPic, autoTakeAPic, manualTakeAPic
        # Open the image file
        print("converting the image now")
        if gotPic:
            if autoTakeAPic:
                with open("autoPicTaken.jpg", "rb") as f:
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
    
            elif manualTakeAPic:
                with open("manualPicTaken.jpg", "rb") as f:
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

    # MAIN class method to execute all MQTT tasks
    def mainFunction(self, client):
        global autoTakeAPic, manualTakeAPic, gotPic
        self.subscribeMQTT(take_pic_topic, client)
        self.publishMQTT(img_topic, client)        
        if self.doneConversion:
            self.publishMQTT(rssi_topic, client)
            self.publishMQTT(metal_waste_topic, client)
            self.publishMQTT(battery_waste_topic, client)
            self.publishMQTT(electronic_waste_topic, client)
            self.publishMQTT(general_dry_waste_topic, client)
            self.publishMQTT(general_wet_waste_topic, client)
            self.publishMQTT(cpu_usage_topic, client)
            self.publishMQTT(ram_usage_topic, client)
            self.publishMQTT(cpu_temp_topic, client)
            manualTakeAPic = False
            autoTakeAPic = False
            gotPic = False
            self.doneConversion = False

# --------------
# MAIN FUNCTION
# --------------
def main(args = None):
    client_class = MQTTClientHandler()
    client = client_class.connectMQTT()

    camera_handler = cameraHandler()
    #gpio_handler = GPIOHandler()
    #serial_handler = serialHandler()

    while True:
        camera_handler.mainFunction()
        #gpio_handler.mainFunction()
        #serial_handler.mainFunction()
        client.loop_start()    
        client_class.mainFunction(client)
        client.loop_stop() 

if __name__ == "__main__":
    main()
