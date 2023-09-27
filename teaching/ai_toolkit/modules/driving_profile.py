import os
import threading

import tensorflow as tf
import numpy as np
import time

from teaching.interface.communication import DataPacket
from ..node import LearningModule


class RLModule(LearningModule):
    def __init__(self, model_path: str, **fed_client_args):
        super(RLModule, self).__init__("rabbitmq", "rabbitmq")
        self.model_path = model_path
        self.fed_client_args = fed_client_args

        self._periodic_sender = None

    def run(self):
        aggregator = Aggregator()
        while True:
            msg = self.receive()
            aggregator.aggregate(msg)
            if aggregator.is_ready():
                profile = self.model.predict(np.asarray([aggregator._batch_data]))
                aggregator.clean()
                final_value = float(np.argmax(profile[0]))
                self.send(
                    DataPacket(
                        topic="prediction.driving_profile.value",
                        body={"driving_profile": final_value},
                    )
                )

    def build(self):
        self._model = tf.keras.models.load_model(self.model_path)
        self._model.summary()
        # TODO: add federated client instantiation


class Aggregator:
    def __init__(self):
        self._namespaces = ["stress", "excitement", "ay", "gz", "speed", "speed_limit"]
        self._batch_data = [None] * len(self._namespaces)

    def aggregate(self, msg):
        if type(msg.body) == list:
            msg.body = msg.body[0]
        msg_keys = msg.body.keys()
        for vkey in msg_keys:
            if vkey in self._namespaces:
                position = self._namespaces.index(vkey)
                self._batch_data[position] = msg.body[vkey]

    def is_ready(self):
        for value in self._batch_data:
            if value is None:
                return False
        return True

    def clean(self):
        self._batch_data = [None] * len(self._namespaces)


def periodic_send_model(lm: LearningModule):
    def aux_fn():
        send_every_t = int(os.getenv("SEND_MODEL_INTERVAL", "5"))
        while True:
            time.sleep(send_every_t)
            lm.fit()
            lm._client.send_model = {
                "model": tf.keras.models.clone_model(lm._model),
                "metadata": {},
            }

    periodic_sender = threading.Thread(target=aux_fn)
    periodic_sender.start()
    return periodic_sender
