import os
from typing import Dict, Optional
from tensorflow import keras
import numpy as np


from ..node import FederatedServer, FederatedClient


class FedAvgServer(FederatedServer):
    def __init__(
        self,
        topic: str,
        epochs: int = 1,
        min_models: int = 2,
        models_path: str = "storage/federated/local",
        model_ext: str = "dat",
    ) -> None:
        super(FedAvgServer, self).__init__(topic)
        self.epochs = epochs
        self.n_before_avg = min_models

        self._storage_path = models_path
        self._client_model_ext = model_ext
        self._client_paths = set()
        os.makedirs(self._storage_path, exist_ok=True)

    def run(self) -> Optional[Dict]:
        for e in self.epochs:
            print(f"Server epoch {e}", flush=True)
            self.send(model=self._curr_model)
            while len(self._client_paths) >= self.n_before_avg:
                msg = self.receive()
                c_path = os.path.join(self._storage_path, f"{msg.service_name}")
                model = msg.body.pop("model")
                msg.to_file(f"{c_path}.dat")
                model.save(f"{c_path}.h5")
                self._client_paths.add(c_path)

            aggregated = keras.models.model_from_json(model.to_json())
            new_weights = [
                np.array(keras.models.load_model(f"{p}.h5").get_weights())
                for p in self._client_paths
            ]
            new_weights = sum(new_weights) / len(
                new_weights
            )  # TODO: edit averaging to take into account dataset size
            aggregated.set_weights(new_weights.tolist())
            self._client_paths = set()
            self._curr_model = aggregated


class FedAvgClient(FederatedClient):
    def __init__(self, topic: str, epochs: int = 1) -> None:
        super(FedAvgClient, self).__init__(topic)
        self.epochs = epochs

    def run(self) -> None:
        for e in self.epochs:
            print(f"Client epoch {e}", flush=True)
            msg = self.receive()
            model = msg.body.pop("model")
            model.fit(**msg.body)
            self.send(model=model)
