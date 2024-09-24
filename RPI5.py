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
import requests
import psutil
import serial
from serial import SerialException
from gpiozero import LED
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
broker = "192.168.43.32"    # For local network application
#broker = "osj-ROG-Strix-G531GT-G531GT"    # For local network application
# broker = "localhost"    # For local network application
port = 1883
client_id = f'python-mqtt-{random.randint(0, 1000)}'

# serial comms parameters
serial_id = "usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0"


# Telegram parameters
BOT_TOKEN = "7299492934:AAGoFfGdU0dZTac3eVQP5byGTK6_98EAhsQ"
chat_id = "-1002259258276"


# camera parameters
FRAME_WIDTH = 640
FRAME_HEIGHT = 480
#cam_id = "usb-Sonix_Technology_Co.__Ltd._USB_2.0_Camera_SN0001-video-index"
AUTOTAKE_PHOTO_DELAY = 60000    # 1 minute delay
WAIT_CHECK_DELAY = 5000
MANUAL_PHOTO_DELAY = 5000
WAIT_UNTIL_DETECTED_DELAY = 7000


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
disable_all_topic = "/group_11/disable_all"

id_topic = f"/group_11/{dustbin_ID}/id"
sensor_topic = f"/group_11/{dustbin_ID}/sensor_details"
waste_topic = f"/group_11/{dustbin_ID}/waste_details"
dustbin_state_topic = f"/group_11/{dustbin_ID}/dustbin_state"


got_metal_waste = 0
got_battery_waste = 0
got_electronic_waste = 0
got_general_dry_waste = 0
got_general_wet_waste = 0

current_metal_waste = 0
current_battery_waste = 0
current_electronic_waste = 0
current_general_dry_waste = 0
current_general_wet_waste = 0

metal_waste_level = 0
battery_waste_level = 0
electronic_waste_level = 0
general_dry_waste_level = 0
general_wet_waste_level = 0

co_level = 0
methane_level = 0
air_quality_level = 0
temperature = 0
humidity = 0
dustbin_weight = 0


# logics
gotPicAuto = False
gotPicManual = False
autoTakeAPic = False
manualTakeAPic = False
continueOp = True
objectDetected = False
callCleaner = False
beginParsing = False
overrideStop = 0 # 0 - no need to stop, 1 - stop the whole thing!
whichBinPartition = 0

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
        self.errMsgOnce = False
        self.count = 0
        self.outsideCount = 0
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
            self.arduinoPort = serial.Serial(f'/dev/serial/by-id/{serial_id}', 115200)
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
        global continueOp, objectDetected, beginParsing, objectDetected, whichBinPartition, callCleaner
        global metal_waste_level, battery_waste_level, electronic_waste_level, general_dry_waste_level, general_wet_waste_level
        global co_level, methane_level, air_quality_level, temperature, humidity, dustbin_weight
        global got_general_dry_waste, got_general_wet_waste, got_battery_waste, got_electronic_waste, got_metal_waste
        global current_battery_waste, current_electronic_waste, current_general_dry_waste, current_general_wet_waste, current_metal_waste

        try:
            incomingSerial = self.arduinoPort.readline()
            dataToString = str(incomingSerial)
            #print(dataToString)

            splitData = dataToString[2:-5].split(":")

            try:
                if int(splitData[0]) == 0:
                    objectDetected = False
                else:
                    objectDetected = True
            except Exception as err:
                print(err)

            try:
                if int(splitData[1]) == 0:
                    continueOp = False
                else:
                    continueOp = True
            except Exception as err:
                print(err)

            try:
                if int(splitData[2]) == 0:
                    callCleaner = False
                else:
                    callCleaner = True
            except Exception as err:
                print(err)

            try:
                levelSplitData = splitData[3].split("/")
                metal_waste_level = float(levelSplitData[0])
                battery_waste_level = float(levelSplitData[1])
                electronic_waste_level = float(levelSplitData[2])
                general_dry_waste_level = float(levelSplitData[3])
                general_wet_waste_level = float(levelSplitData[4])
            except Exception as err:
                print(err)

            try:
                gotWasteSplitData = splitData[4].split("/")
                got_metal_waste = int(gotWasteSplitData[0])
                got_battery_waste = int(gotWasteSplitData[1])
                got_electronic_waste = int(gotWasteSplitData[2])
                got_general_dry_waste = int(gotWasteSplitData[3])
                got_general_wet_waste = int(gotWasteSplitData[4])
            except Exception as err:
                print(err)
            
            try:
                currentWasteSplitData = splitData[5].split("/")
                current_metal_waste = int(currentWasteSplitData[0])
                current_battery_waste = int(currentWasteSplitData[1])
                current_electronic_waste = int(currentWasteSplitData[2])
                current_general_dry_waste = int(currentWasteSplitData[3])
                current_general_wet_waste = int(currentWasteSplitData[4])
            except Exception as err:
                print(err)


            try:
            
                sensorSplitData = splitData[6].split("/")
                co_level = float(sensorSplitData[0])
                methane_level = float(sensorSplitData[1])
                air_quality_level = float(sensorSplitData[2])
                temperature = float(sensorSplitData[3])
                humidity = float(sensorSplitData[4])
                dustbin_weight = float(sensorSplitData[5])

            except Exception as err:
                print(err)


        except SerialException or OSError:
            self.disconnected = True
            self.reopenPort()


    def serialTransmit(self):
        global autoTakeAPic, gotPicAuto, gotPicManual, objectDetected, detectedTrash, whichBinPartition
        #self.whichBinPartition = 5
        self.outsideCount += 1
        #print(f"outsideCount: {self.outsideCount}")
        if (autoTakeAPic or objectDetected) and gotPicAuto:
            if len(detectedTrash) != 0:
                print(detectedTrash[0])
                if detectedTrash[0] == "Metal":
                    whichBinPartition = 2
                    
                elif detectedTrash[0] == "battery":
                    whichBinPartition = 1

                elif detectedTrash[0] == "Electronic Devices":
                    whichBinPartition = 5

                elif detectedTrash[0] == "General-Dry-Waste":
                    whichBinPartition = 4

                elif detectedTrash[0] == "General Wet Waste":
                    whichBinPartition = 3
            else:
                print("No trash detected!")
                whichBinPartition = 0

            try:
                self.arduinoPort.write(f'{whichBinPartition} {overrideStop}\r'.encode())
                self.disconnected = False
                print(f"{whichBinPartition} {overrideStop}")

            except SerialException:
                self.disconnected = True
                self.reopenPort()

            self.count += 1
            #print(f"insideCount: {self.count}")

        elif objectDetected == 0:
            whichBinPartition = 0

    # -------------
    # TASK FUNCTION
    # -------------

    def mainFunction(self):
        self.serialTransmit()
        self.serialReceive()





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
        self.prevWaitCheckTime = self.currentMillis()

        self.result = None
        self.image = None
        self.processedImage = None
        self.doneDetection = False
        self.takeOnce = False

        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)

        self.model = YOLO("v22.pt")

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
        global gotPicManual, gotPicAuto, manualTakeAPic, autoTakeAPic, objectDetected

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
                gotPicManual = True

            # otherwise, print an error message
            else:
                gotPicManual = False
                print("No manual image detected. Please try again")

        # if we received a request to take a picture
        elif (autoTakeAPic or objectDetected):
            #print("taking photo of the trash...")
            # if there is image data, save it as autoPicTaken.JPG in the current directory
            if self.doneDetection:
                myFile = Path(f"autoPicTaken.jpg")
                if myFile.is_file():
                    try:
                        os.remove("autoPicTaken.jpg")
                    except:
                        pass
                cv2.imwrite(f"autoPicTaken.jpg", self.processedImage)
                gotPicAuto = True
                self.doneDetection = False

            # otherwise, print an error message
            else:
                gotPicAuto = False
                #print("No auto image detected. Please try again")


    # since Raspberry Pi 5 is still not a good computer to do live detection, only image source will be used to do detection
    def detectTrashFromImage(self):
        global autoTakeAPic, objectDetected, detectedTrash

        # detect the objects in the color images
        if (autoTakeAPic or objectDetected) and whichBinPartition == 0 and self.result:
            if (self.currentMillis() - self.prevWaitCheckTime >= WAIT_CHECK_DELAY):
                print("detecting the trash...")
                #print("I got a picture!, processing it now")
                detectedTrash = []
                results = self.model.predict(source = self.image, show_boxes = False, verbose = False, show = False, conf = 0.65, max_det = 3)[0]
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
                self.prevWaitCheckTime = self.currentMillis()

        else:
            self.prevWaitCheckTime = self.currentMillis()

    def mainFunction(self):
        global autoTakeAPic, objectDetected
        if continueOp:
            # if there is no trash for quite some time already, automatically take a photo every time the set period of 1 minute
            if self.currentMillis() - self.prevInterruptTime >= AUTOTAKE_PHOTO_DELAY:
                print("Timeout triggered, taking a photo with detections...")
                autoTakeAPic = True
                self.prevInterruptTime = self.currentMillis()
            elif objectDetected:
                self.prevInterruptTime = self.currentMillis()
            self.capturePicture()
            self.detectTrashFromImage()



# -------------------------
# -------------------------
#
# TELEGRAM HANDLER CLASS
#
# -------------------------
# -------------------------

class telegramHandler:
    # initialize the class and the GPIO
    def __init__(self):
        super().__init__()
        self.sendTelegramOnce = False
        self.msg = ""
        self.url = ""

    def msgCheck(self):
        if callCleaner and not self.sendTelegramOnce:
            self.sendTelegramOnce = True
            if current_battery_waste >= 5 and current_general_wet_waste >= 20:
                self.msg = f"Battery waste and general wet waste quantities exceeded safe limits! Please clean bin {dustbin_ID}"            
            elif current_battery_waste >= 5:
                self.msg = f"Battery waste quantity exceed safe limit! Please clean bin {dustbin_ID}"
            elif current_general_wet_waste >= 20:
                self.msg = f"General wet waste quantity exceed safe limit! Please clean bin {dustbin_ID}"
            elif dustbin_weight >= 20.0:
                self.msg = f"Dustbin weight exceeded 20 kg! Current reading is {dustbin_weight} kg. Please clean bin {dustbin_ID}"
            elif metal_waste_level == 1.00:
                self.msg = f"Metal waste partition is full! Please clean bin {dustbin_ID}"
            elif battery_waste_level == 1.00:
                self.msg = f"Battery waste partition is full! Please clean bin {dustbin_ID}"
            elif electronic_waste_level == 1.00:
                self.msg = f"Electronic waste partition is full! Please clean bin {dustbin_ID}"
            elif general_dry_waste_level == 1.00:
                self.msg = f"General dry waste partition is full! Please clean bin {dustbin_ID}"
            elif general_wet_waste_level == 1.00:
                self.msg = f"General wet waste partition is full! Please clean bin {dustbin_ID}"
            elif co_level >= 20.0:
                self.msg = f"CO level exceeds safe limits! Please service bin {dustbin_ID}"
            elif methane_level >= 20.0:
                self.msg = f"Methane level exceeds safe limits! Please service bin {dustbin_ID}"
            elif air_quality_level >= 20.0:
                self.msg = f"Air quality is bad! Please service bin {dustbin_ID}"
            elif temperature >= 45.0:
                self.msg = f"Dustbin is very hot! Please service bin {dustbin_ID}"
            else:
                self.msg = f"Unknown error, please service bin {dustbin_ID}"

            self.url = f"https://api.telegram.org/bot{BOT_TOKEN}/sendMessage?chat_id={chat_id}&text={self.msg}"
            r = requests.get(self.url)
            print(r.json())

        elif not callCleaner:
            self.sendTelegramOnce = False

    def mainFunction(self):
        if continueOp:
            self.msgCheck()




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
        self.relay = LED(17)
    # --------------------
    # TASK FUNCTION
    # --------------------

    # class method to control the LED strip
    def controlLEDStrip(self):
        global manualTakeAPic, autoTakeAPic, objectDetected

        if autoTakeAPic or manualTakeAPic or objectDetected:
            self.relay.on()
        else:
            self.relay.off()

    def mainFunction(self):
        if continueOp:
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
        self.prevManualPhotoMillis = self.currentMillis()
        self.prevWaitUntilDetectedMillis = self.currentMillis()
        self.bypassDetect = False
        self.doneConversion = False
        self.msgToPub = None
        self.prevSendMillis = self.currentMillis()
        self.waitForData = False
        self.sendDetectedOnce = False
    # ------------------------------------
    # MQTT CONNECTION HANDLING FUNCTIONS
    # ------------------------------------

    # handles connection to the Mosquitto MQTT
    def onConnect(self, client, userdata, flags, rc, properties = None):
        if rc == 0:
            print("Connected to Mosquitto MQTT Broker")
        else:
            print("Failed to connect to Mosquitto MQTT broker, return code %d\n", rc)

        #print(f"rc: {rc}")

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
        #client.tls_set()
        client.username_pw_set(username="Group_11", password="guoenjustinsoojack")
        client.connect(broker, port, 60)
        return client
    

    # ------------------------------------
    # MQTT MESSAGE HANDLING FUNCTIONS
    # ------------------------------------

    # class method to handle MQTT data coming from NodeRED or other sources
    def on_message(self, client, userdata, msg):
        global autoTakeAPic, manualTakeAPic, continueOp
        print(f"Received `{msg.payload.decode()}` from `{msg.topic}` topic")


        # if we received a request to take a photo
        if msg.topic == '/group_11/take_picture':
            print("Calling Camera Handler to take photo...")
            manualTakeAPic = msg.payload.decode()


        elif msg.topic == '/group_11/disable_all':
            if msg.payload.decode() == "true":
                print("Dustbin is disabled!")
                continueOp = False
            elif msg.payload.decode() == "false":
                print("Dustbin is enabled!")
                continueOp = True

    # class method to handle message publishing task
    def publishMQTT(self, topic, client):
        global autoTakeAPic, manualTakeAPic, gotPicManual, gotPicAuto

        # if we received a request to take a picture, receive the base64 code of the picture
        if topic == img_topic and (gotPicManual or gotPicAuto):
            image = str(self.imgToBase64())
            print("converting the image...")
            jsonData = [
                {
                    "image": image[2:-1]
                },
                {
                    "dustbin_id": dustbin_ID
                }
            ]

            self.msgToPub = json.dumps(jsonData)

            self.doneConversion = True

        elif topic == waste_topic:
            jsonData = [
                {
                    "dustbin_metal_waste": got_metal_waste,
                    "dustbin_battery_waste": got_battery_waste,
                    "dustbin_electronic_waste": got_electronic_waste,
                    "dustbin_general_dry_waste": got_general_dry_waste,
                    "dustbin_general_wet_waste": got_general_wet_waste,
                    "waste_detected": detectedTrash[0] if len(detectedTrash) != 0 else "None",
                    "current_metal_waste": current_metal_waste,
                    "current_battery_waste": current_battery_waste,
                    "current_electronic_waste": current_electronic_waste,
                    "current_general_dry_waste": current_general_dry_waste,
                    "current_general_wet_waste": current_general_wet_waste,
                    "level_metal_waste": "Full" if metal_waste_level == 1 else "Not full",
                    "level_battery_waste": "Full" if battery_waste_level == 1 else "Not full",
                    "level_electronic_waste": "Full" if electronic_waste_level == 1 else "Not full",
                    "level_general_dry_waste": "Full" if general_dry_waste_level == 1 else "Not full",
                    "level_general_wet_waste": "Full" if general_wet_waste_level == 1 else "Not full",                    
                },
                {
                    "dustbin_id": dustbin_ID
                }
            ]    
            
            self.msgToPub = json.dumps(jsonData)

        elif topic == dustbin_state_topic:
            try:
                rssi = float(self.getRSSI("wlan0"))
            except:
                print("No RSSI value obtained!")
                rssi = 8888.0   # send an impossible value

            try:
                cpuUsage = self.getCPUUsage()
            except:
                print("No CPU Usage value obtained!")
                cpuUsage = 8888.0   # send an impossible value

            try:
                ramUsage = self.getRAMUsage()
            except:
                print("No RAM Usage value obtained!")
                ramUsage = 8888.0   # send an impossible value

            try:
                cpuTemp = self.getCPUTemp()
            except:
                print("No RAM Usage value obtained!")
                cpuTemp = 8888.0    # send an impossible value


            jsonData = [
                {
                    "dustbin_op": "Running" if continueOp else "Stopped",
                    "dustbin_op_code": 1 if continueOp else 0,
                    "dustbin_rssi": rssi,
                    "dustbin_cpu_usage": cpuUsage,
                    "dustbin_ram_usage": ramUsage,
                    "dustbin_cpu_temp": cpuTemp                    
                },
                {
                    "dustbin_id": dustbin_ID,
                }
            ]
        
            self.msgToPub = json.dumps(jsonData)
        
        elif topic == sensor_topic:
            jsonData = [
                {
                    "dustbin_co_lvl": co_level,
                    "dustbin_methane_lvl": methane_level,
                    "dustbin_air_quality_lvl": air_quality_level,
                    "dustbin_temperature": temperature,
                    "dustbin_humidity": humidity,
                    "dustbin_weight": dustbin_weight                 
                },
                {
                    "dustbin_id": dustbin_ID,
                }
            ]
            self.msgToPub = json.dumps(jsonData)

        elif topic == id_topic:
            self.msgToPub = dustbin_ID

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
        return psutil.sensors_temperatures()['cpu_thermal'][0].current

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
        global gotPicAuto, autoTakeAPic, manualTakeAPic, objectDetected
        # Open the image file
        #print("converting the image now")
        if (autoTakeAPic or objectDetected) and gotPicAuto:
            with open("autoPicTaken.jpg", "rb") as f:
                buffer = BytesIO()
                image = Image.open(f)

                # resize the image so that the base64 code is not too long
                # however this will result in blurry image
                # hence the scaling is removed
                width, height = image.size
                new_size = (width, height)
                resized_image = image.resize(new_size)
                resized_image.save(buffer, format="JPEG")
                encoded_image = base64.b64encode(buffer.getvalue())
                #print(str(encoded_image)[2:-1])
                return encoded_image

        elif manualTakeAPic and gotPicManual:
            with open("manualPicTaken.jpg", "rb") as f:
                buffer = BytesIO()
                image = Image.open(f)

                # resize the image so that the base64 code is not too long
                width, height = image.size
                new_size = (width, height)
                resized_image = image.resize(new_size)
                resized_image.save(buffer, format="JPEG")
                encoded_image = base64.b64encode(buffer.getvalue())
                #print(str(encoded_image)[2:-1])
                return encoded_image

    def currentMillis(self):
        return round(time.time() * 1000)


    # MAIN class method to execute all MQTT tasks
    def mainFunction(self, client):
        global autoTakeAPic, manualTakeAPic, gotPicAuto, gotPicManual
        
        self.subscribeMQTT(take_pic_topic, client)
        self.subscribeMQTT(disable_all_topic, client)

        if objectDetected and not self.sendDetectedOnce:
            #print(got_battery_waste + got_metal_waste + got_electronic_waste + got_general_dry_waste + got_general_wet_waste)
            if not self.waitForData and gotPicAuto:
                self.prevWaitUntilDetectedMillis = self.currentMillis()
                self.publishMQTT(img_topic, client)
                self.waitForData = True


            if len(detectedTrash) != 0 and got_battery_waste + got_electronic_waste + got_general_dry_waste + got_general_wet_waste + got_metal_waste != 0:
                print("I see trash! waiting for confirmation...")

                # print(self.waitForData)
                # print(self.doneConversion)
                # print(gotPicAuto)
                # print(self.sendDetectedOnce)

                if self.doneConversion:
                    self.publishMQTT(waste_topic, client)
                    self.publishMQTT(sensor_topic, client)
                    self.publishMQTT(dustbin_state_topic, client)
                    
                    self.sendDetectedOnce = True
                    gotPicAuto = False
                    self.bypassDetect = False
                    #self.doneConversion = False
                    self.waitForData = False                
                    
            else:
                print("waiting for timeout of no detections...")
                # if there is no data after 1 second, we will proceed with sending the data
                if self.currentMillis() - self.prevWaitUntilDetectedMillis >= WAIT_UNTIL_DETECTED_DELAY:
                    if got_battery_waste + got_electronic_waste + got_general_dry_waste + got_general_wet_waste + got_metal_waste != 0:
                        self.publishMQTT(waste_topic, client)
                        self.publishMQTT(sensor_topic, client)
                        self.publishMQTT(dustbin_state_topic, client)
                        self.prevWaitUntilDetectedMillis = self.currentMillis()
                        self.sendDetectedOnce = True
                        gotPicAuto = False
                        self.bypassDetect = False
                        #self.doneConversion = False
                        self.waitForData = False        


        elif autoTakeAPic:
            if not self.waitForData and gotPicAuto:
                self.publishMQTT(img_topic, client)
                self.waitForData = True

            if self.doneConversion:
                self.publishMQTT(waste_topic, client)
                self.publishMQTT(sensor_topic, client)
                self.publishMQTT(dustbin_state_topic, client)
                autoTakeAPic = False
                self.waitForData = False
                gotPicAuto = False
                #self.doneConversion = False

        elif manualTakeAPic:
            self.publishMQTT(img_topic, client)
            self.publishMQTT(waste_topic, client)
            self.publishMQTT(sensor_topic, client)
            self.publishMQTT(dustbin_state_topic, client)
            
            if self.currentMillis() - self.prevManualPhotoMillis >= MANUAL_PHOTO_DELAY:
                manualTakeAPic = False
                gotPicManual = False
                #self.doneConversion = False                
                self.prevManualPhotoMillis = self.currentMillis()

        else:
            if not objectDetected:
                self.sendDetectedOnce = False
            self.waitForData = False
            self.doneConversion = False
            self.prevWaitUntilDetectedMillis = self.currentMillis()
            self.prevManualPhotoMillis = self.currentMillis()


# --------------
# MAIN FUNCTION
# --------------
def main(args = None):
    client_class = MQTTClientHandler()
    client = client_class.connectMQTT()

    camera_handler = cameraHandler()
    gpio_handler = GPIOHandler()
    serial_handler = serialHandler()
    telegram_handler = telegramHandler()

    while True:
        camera_handler.mainFunction()
        gpio_handler.mainFunction()
        serial_handler.mainFunction()
        telegram_handler.mainFunction()
        client.loop_start()    
        client_class.mainFunction(client)
        client.loop_stop() 

if __name__ == "__main__":
    main()
