import os
import time
from typing import Dict, Optional
from tensorflow import keras
import numpy as np


from ..node import FederatedServer, FederatedClient


class DummyServer(FederatedServer):
    def __init__(
        self,
        topic: str,
        epochs: int = 1,
        min_models: int = 2,
        models_path: str = "storage/federated/local",
        model_ext: str = "dat",
    ) -> None:
        super(DummyServer, self).__init__(topic)
        self.epochs = epochs
        self.n_before_avg = min_models

        self._storage_path = models_path
        self._client_model_ext = model_ext
        self._client_paths = set()
        os.makedirs(self._storage_path, exist_ok=True)

    def run(self) -> Optional[Dict]:
        for e in self.epochs:
            self.send(model=self._curr_model)
            print(f"Server epoch {e}", flush=True)
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
            new_weights = sum(new_weights) / len(new_weights)
            aggregated.set_weights(new_weights.tolist())
            self._curr_model = aggregated
            self._client_paths = set()


class DummyClient(FederatedClient):
    def __init__(self, topic: str, send_interval: float = 5.0) -> None:
        super(DummyClient, self).__init__(topic)
        self.send_interval = send_interval

    def run(self):
        while True:
            msg = self.receive()
            model = keras.models.model_from_json(model.to_json())
            time.sleep(self.send_interval)

            lm._client.send_model = {
                "model": tf.keras.models.clone_model(lm._model),
                "metadata": {},
            }
