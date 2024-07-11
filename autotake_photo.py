import cv2 as cv
from pathlib import Path
import time

fileCount = 0
camera = '/dev/v4l/by-id/usb-Sonix_Technology_Co.__Ltd._USB_2.0_Camera_SN0001-video-index0'
previousMillis = 0

DELAY = 1000

def currentMillis():
    return round(time.time() * 1000)


def main(args=None):
    global fileCount, previousMillis, camera
    cap = cv.VideoCapture(camera)
    cap.set(cv.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv.CAP_PROP_FRAME_HEIGHT, 480)    
    #cap.set(cv.CAP_PROP_EXPOSURE, 1000)
    while True:


        result, image = cap.read()

        if result:
            cv.imshow("welp", image)

            if currentMillis() - previousMillis >= DELAY:
                myFile = Path(f"./dataset/trash{fileCount}.png")
                if not myFile.is_file():
                    cv.imwrite(f"./dataset/trash{fileCount}.png", image)
                    fileCount += 1
                else:
                    while True:
                        myFile = Path(f"./dataset/trash{fileCount}.png")
                        if myFile.is_file():
                            fileCount += 1
                        else:
                            break
                previousMillis = currentMillis()
        else:
            print("No image detected. Please try again")
        
        

        if cv.waitKey(33) == ord('q'):
            cv.destroyAllWindows()
            break


if __name__ == "__main__":
    main()