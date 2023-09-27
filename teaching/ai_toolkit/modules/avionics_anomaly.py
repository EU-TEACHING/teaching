from typing import Dict
import tensorflow as tf
import numpy as np

from teaching.interface.communication.packet import DataPacket
from ..node import LearningModule
from ..training.federated import get_client


class AvionicsAnomalyDetector(LearningModule):
    def __init__(self, model_path: str, **fed_client_args):
        super(AvionicsAnomalyDetector, self).__init__("rabbitmq", "rabbitmq")
        self.model_path = model_path
        self.fed_client_args = fed_client_args

    def run(self):
        aggregator = Aggregator()
        while True:
            msg = self.receive()
            aggregator.aggregate(msg)
            if aggregator.is_ready():
                mitigation_plan = self._model.predict(
                    np.asarray([aggregator._batch_data])
                )
                aggregator.clean()
                final_value = float(np.argmax(mitigation_plan[0]))
                self.send(
                    DataPacket(
                        topic="prediction.mitigation_plan.value",
                        body={"mitigation_plan": final_value},
                    )
                )

    def build(self):
        self._model = tf.keras.models.load_model(self.model_path)
        self._model.summary()
        # TODO: add federated client instantiation


class Aggregator:
    def __init__(self):
        self._namespaces = [
            "cpu_1",
            "cpu_2",
            "cpu_3",
            "cpu_4",
            "network",
            "buffer",
            "hdd",
            "cache",
            "param_1",
            "param_2",
            "anomaly",
            "mitigation",
        ]
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
