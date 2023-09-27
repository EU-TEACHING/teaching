from tensorflow import keras
from keras.applications.mobilenet_v2 import (
    preprocess_input,
    decode_predictions,
)
import numpy as np

from ..node import LearningModule


class ORModule(LearningModule):
    def __init__(self):
        super(ORModule, self).__init__(None, "rabbitmq")

    def run(self):
        while True:
            msg = self.receive()
            np_img = np.asarray(eval(msg.body["img"]), dtype="uint8").reshape(
                224, 224, 3
            )
            print("image received!")

            img_batch = np.expand_dims(np_img, 0)
            pred = self.model.predict(preprocess_input(img_batch))
            print("Predictions: ", decode_predictions(pred))

    def build(self):
        self._model = keras.applications.MobileNetV2(
            weights="imagenet", include_top=True
        )
        self.model.trainable = False
        self.model.summary()
