import os
from typing import Dict, List, Optional
import tensorflow as tf
import numpy as np

from teaching.interface.communication import DataPacket
from ..node import LearningModule
from ..models.esn import ESN
from ..data import TEACHINGDataset
from teaching.ai_toolkit.training import get_trainable


class StressModule(LearningModule):
    def __init__(
        self,
        topics: List[str],
        output_topic: str = "prediction.stress.value",
        feature_keys: List[str] = ["eda"],
        target_keys: Optional[List[str]] = None,
        model_path: str = None,
        input_size: int = 1,
        layers: int = 1,
        units: int = 20,
        connectivity: float = 1.0,
        leaky: float = 0.8,
        spectral_radius: float = 0.9,
        n_classes: int = 1,
        calibration_steps: int = 0,
        trainable_args: Optional[Dict] = None,
    ):
        super(StressModule, self).__init__("rabbitmq", "rabbitmq")
        self.topics = topics
        self.feature_keys = feature_keys
        self.target_keys = target_keys
        self.model_path = model_path
        self.output_topic = output_topic
        if self.model_path is None:
            self.input_size = input_size
            self.layers = layers
            self.units = units
            self.connectivity = connectivity
            self.leaky = leaky
            self.spectral_radius = spectral_radius
            self.n_classes = n_classes

        self.calibration_steps = calibration_steps
        self.trainable_args = trainable_args
        self._trainable = None
        self._data = None

    def run(self):
        mean, std = self.calibrate()
        while True:
            msg = self.receive()
            is_list = isinstance(msg.body, List)
            body = msg.body if is_list else [msg.body]
            stress = [] if is_list else None
            for body_t in body:
                x = tf.constant([[self._data(body_t, self.feature_keys)]])
                pred = {"stress": float(tf.squeeze(self._model(x)).numpy())}
                if is_list:
                    stress.append(pred)
                else:
                    stress = pred

            self.send(
                DataPacket(
                    topic=self.output_topic,
                    timestamp=msg.timestamp,
                    body=stress,
                )
            )

    def calibrate(self):
        if self.calibration_steps < 2:
            return 0, 1
        n_received = 0
        buffer = []
        while n_received < self.calibration_steps:
            msg = self.receive()
            if isinstance(msg.body, List):
                to_append = [v["eda"] for v in msg.body]
            else:
                to_append = [msg.body["eda"]]
            n_received += len(to_append)
            buffer += to_append
        return np.mean(buffer), np.std(buffer)

    def build(self):
        self._model = self.build_model()
        self._data = TEACHINGDataset(
            self.feature_keys, self.target_keys, min_size=9000, max_size=10500
        )

        for t in self.topics:
            self.consumer.subscribe(t)

        if self.trainable_args is not None:
            self._trainable = get_trainable(self.trainable_args.pop("type"))
            self._trainable = self._trainable(lm=self, **self.trainable_args)
            self._trainable.start()

    def build_model(self):
        if self.model_path is not None and os.path.exists(self.model_path):
            self._model = tf.keras.models.load_model(
                self.model_path, custom_objects={"ESN": ESN}
            )
        else:
            inputs = tf.keras.Input(batch_shape=(1, 1, self.input_size))
            for i in range(self.layers):
                x = ESN(
                    units=self.units,
                    leaky=self.leaky,
                    spectral_radius=self.spectral_radius,
                    connectivity_input=self.connectivity,
                    return_sequences=True,
                    stateful=True,
                )(inputs if i == 0 else x)
            outputs = tf.keras.layers.Dense(
                self.n_classes,
                activation=("sigmoid" if self.n_classes <= 2 else "softmax"),
            )(x)
            self._model = tf.keras.Model(
                inputs=inputs, outputs=outputs, name="stress_model"
            )
        self._model.summary()
        return self._model
