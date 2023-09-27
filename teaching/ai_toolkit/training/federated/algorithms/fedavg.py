import time
import teaching
import numpy as np
from typing import Dict, Optional
from teaching.ai_toolkit.node import LearningModule
from teaching.ai_toolkit.training.federated.serialization import serialize_model
from teaching.interface.communication.packet import DataPacket

from ..node import FederatedServer, FederatedClient


class FedAvgServer(FederatedServer):
    def __init__(
        self, topic: str, model_path: str, epochs: int = 1, n_before_avg: int = 2
    ) -> None:
        super(FedAvgServer, self).__init__(topic, model_path)
        self.epochs = epochs
        self.n_before_avg = n_before_avg

    def run(self) -> Optional[Dict]:
        for e in range(self.epochs):
            print(f"Server epoch {e}", flush=True)
            self.send(
                DataPacket(
                    topic=self.output_topic, body={"model": serialize_model(self.model)}
                )
            )
            received = 0
            params, weights = None, 0
            while received < self.n_before_avg:
                msg = self.receive()
                curr_params = msg.body["model"]["weights"]
                curr_weights = msg.body["metadata"]["n_samples"]
                if params is None:
                    params = [np.array(p) * curr_weights for p in curr_params]
                else:
                    params = [
                        params[i] + np.array(curr_params[i]) * curr_weights
                        for i in range(len(params))
                    ]
                weights += curr_weights
                received += 1
            params = [(p / weights) for p in params]
            self.model.set_weights(params)
            self.send(
                DataPacket(
                    topic=self.output_topic, body={"model": serialize_model(self.model)}
                )
            )
            self.model.save(self.model_path + "_new")


class FedAvgClient(FederatedClient):
    def __init__(
        self,
        lm: LearningModule,
        topic: str,
        rounds: int = 1,
        epochs: int = 3,
        loss: str = "sparse_categorical_crossentropy",
    ) -> None:
        super(FedAvgClient, self).__init__(lm, topic)
        self.rounds = rounds
        self.epochs = epochs
        self.loss = loss

    def run(self) -> None:
        while not self.data.ready:
            print(
                f"Waiting for data, missing {self.data.min_size - len(self.data)} data points...",
                flush=True,
            )
            time.sleep(3)
        x, y = self.data.samples
        for round in range(self.rounds):
            print(f"Client {teaching.SERVICE_NAME} epoch {round}", flush=True)
            msg = self.receive()
            self.model = msg.body.pop("model")
            self.model.compile(
                optimizer="adam",
                loss=self.loss,
                metrics=["accuracy"],
            )
            results = self.model.evaluate(x, y, batch_size=1, verbose=0)
            print(
                f"Client {teaching.SERVICE_NAME} preliminary evaluation on round {round}: Loss = {results[0]} -- Accuracy = {results[1]}",
                flush=True,
            )
            self.model.fit(
                x, y, epochs=self.epochs, batch_size=1, shuffle=False, verbose=0
            )
            self.send(model=self.model, n_samples=len(self.data))

        msg = self.receive()
        results = self.model.evaluate(x, y, batch_size=1, verbose=0)
        print(
            f"Client {teaching.SERVICE_NAME} evaluation of global model: Loss = {results[0]} -- Accuracy = {results[1]}",
            flush=True,
        )
        y = self.model.predict(x, batch_size=1, verbose=0)
        y = np.squeeze(y).tolist()
        for curr_y in y:
            self._lm.producer.send(
                DataPacket(
                    topic=self._lm.output_topic,
                    body={"stress": curr_y},
                ),
            )
