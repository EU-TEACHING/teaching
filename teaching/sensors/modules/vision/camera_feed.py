import os
import time
from PIL import Image
import numpy as np

from ...node import SensorModule
from teaching.interface.communication import DataPacket


class CameraFeed(SensorModule):
    def __init__(self, output_topic: str, capture_delay: float):
        super().__init__()
        self.img_path = "/app/storage/current_img.jpg"
        self.output_topic = output_topic
        self.capture_delay = capture_delay
        print("Starting RPI Camera Module Service...")

    def run(self):
        old_img = None
        while not os.path.exists(self.img_path):
            print("path does not exists!")
            time.sleep(0.1)

        while True:
            ### create still image from camera and load it
            # os.system("libcamera-still -n -o {} --vflip --hflip".format(self.img_path))
            #
            img = Image.open(self.img_path)
            np_img = np.asarray(img)

            if old_img is None or not np.array_equal(old_img, np_img):
                print("New image found!")
            old_img = np_img

            print("Picture captured!")
            ### send package
            img_msg = str(np_img.flatten().tolist())
            # print("msg to send: ", img_msg)
            #  to convert them back: np.asarray(eval(img_msg), dtype="uint8").reshape(224, 224, 3)
            self.send(DataPacket(topic=self.output_topic, body={"img": img_msg}))
            print("Picture sent!")

            ### add delay
            time.sleep(self.capture_delay)
