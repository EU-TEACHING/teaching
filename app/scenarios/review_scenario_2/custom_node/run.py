import yaml
import teaching

from tensorflow import keras
import tensorflow as tf
from typing import List
from teaching.interface.communication import DataPacket
from teaching.ai_toolkit.node import LearningModule
from teaching.ai_toolkit.data import TEACHINGDataset


class CustomModule(LearningModule):
    def __init__(
        self,
        topics: List[str],
        feature_keys: List[str] = None,
    ):
        super(CustomModule, self).__init__("rabbitmq", "rabbitmq")
        self.topics = topics
        self.feature_keys = feature_keys

    def run(self):
        while True:
            msg = self.receive()
            is_list = isinstance(msg.body, List)
            body = msg.body if is_list else [msg.body]
            stress = [] if is_list else None
            for body_t in body:
                x = tf.constant([self._data(body_t, self.feature_keys)])
                pred = {"value": float(tf.squeeze(self._model(x)).numpy())}
                if is_list:
                    stress.append(pred)
                else:
                    stress = pred

            self.send(
                DataPacket(
                    topic="prediction.custom.value",
                    timestamp=msg.timestamp,
                    body=stress,
                )
            )

    def build(self):
        self._model = keras.Sequential(
            [
                keras.layers.InputLayer(input_shape=(8,)),
                keras.layers.Dense(1, activation="sigmoid"),
            ]
        )
        for t in self.topics:
            self.consumer.subscribe(t)
        self._data = TEACHINGDataset(self.feature_keys)


if __name__ == "__main__":
    config = yaml.safe_load(open("config.yml", "r"))[teaching.SERVICE_NAME]
    module = CustomModule(**config)
    module.build()
    module.run()
