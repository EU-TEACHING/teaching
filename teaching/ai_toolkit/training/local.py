import copy
import threading
from typing import List, Optional
from teaching.ai_toolkit.data import TEACHINGDataset

from teaching.ai_toolkit.node import LearningModule

# from .data import get_dataset


class ModuleTrainer:
    def __init__(self, lm: LearningModule, **config) -> None:
        self._lm = lm
        self._model = None
        self._config = config
        self._train_condition = False
        self._training_thread = None

    def start_training(self) -> None:
        self.train_condition = True
        self._training_thread = threading.Thread(
            target=self.train, kwargs=self._config["train_args"]
        )
        self._training_thread.start()

    def train(self, **train_args) -> None:
        raise NotImplementedError

    def stop_training(self) -> None:
        self.train_condition = False
        self._training_thread.join()

    @property
    def model(self):
        if self._model is None:
            self._model = copy.deepcopy(self._lm.model)
        return self._model

    @property
    def data(self) -> TEACHINGDataset:
        return self._lm._data


class KerasTrainer(ModuleTrainer):
    def __init__(self, lm: LearningModule, **config) -> None:
        super().__init__(lm, **config)

    def train(
        self, feature_keys: List[str], target_keys: Optional[List[str]], **train_args
    ) -> None:
        x, y = self.data.get_samples(feature_keys, target_keys)
        self._lm.fit(x, y, **train_args)
        self.train_condition = False
