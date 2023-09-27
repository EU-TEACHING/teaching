import threading
from typing import Optional
import keras
import teaching
from teaching.ai_toolkit.data import TEACHINGDataset
from teaching.ai_toolkit.models.esn import ESN

from teaching.interface import TEACHINGNode
from teaching.interface.communication import DataPacket
from .serialization import (
    serialize_model,
    deserialize_model,
)

from teaching.ai_toolkit.node import LearningModule


class FederatedClient(TEACHINGNode):
    def __init__(self, lm: LearningModule, topic: str) -> None:
        super().__init__(produce_backend="kafka", consume_backend="kafka")
        self.topic = topic
        self._lm = lm
        self._model = None
        self._client_thread = None

    def start(self):
        self._client_thread = threading.Thread(target=self.run)
        self.consumer.subscribe(self.input_topic)
        self._client_thread.start()

    def run(self):
        raise NotImplementedError

    def stop(self):
        self._client_thread.join()

    def send(self, model: keras.Model, **metadata):
        super().send(
            DataPacket(
                topic=self.output_topic,
                body={"model": serialize_model(model), "metadata": metadata},
            )
        )
        print(
            f"Client {teaching.SERVICE_NAME} sent a new local model.",
            flush=True,
        )

    def receive(self, timeout: Optional[float] = None) -> DataPacket:
        packet = super().receive(timeout=timeout)
        if packet is not None and "model" in packet.body:
            packet.body["model"] = deserialize_model(packet.body["model"])
            print(
                f"Client {teaching.SERVICE_NAME} is receiving a new global model.",
                flush=True,
            )
        return packet

    @property
    def input_topic(self) -> str:
        return f"{self.topic}.global"

    @property
    def output_topic(self) -> str:
        return f"{self.topic}.local"

    @property
    def model(self) -> keras.Model:
        return self._model

    @model.setter
    def model(self, new_model: keras.Model):
        self._model = new_model

    @property
    def data(self) -> TEACHINGDataset:
        return self._lm._data


class FederatedServer(TEACHINGNode):
    def __init__(self, topic: str, model_path: str) -> None:
        super().__init__(produce_backend="kafka", consume_backend="kafka")
        self.topic = topic
        self.model_path = model_path

        self._model = None

    def run(self):
        raise NotImplementedError

    def build(self):
        self.consumer.subscribe(self.input_topic)
        try:
            self._model = keras.models.load_model(self.model_path)
        except ValueError:
            self._model = keras.models.load_model(
                self.model_path, custom_objects={"ESN": ESN}
            )

    def send(self, msg: DataPacket):
        super().send(msg)
        print(
            f"Server {teaching.SERVICE_NAME} is broadcasting a new global model",
            flush=True,
        )

    def receive(self, timeout: Optional[float] = None) -> DataPacket:
        packet = super().receive(timeout=timeout)
        if packet is not None:
            print(
                f"Server received a local model from {packet.service_name}",
                flush=True,
            )
        return packet

    @property
    def input_topic(self) -> str:
        return f"{self.topic}.local"

    @property
    def output_topic(self) -> str:
        return f"{self.topic}.global"

    @property
    def model(self) -> keras.Model:
        return self._model

    @model.setter
    def model(self, new_model: keras.Model):
        self._model = new_model
