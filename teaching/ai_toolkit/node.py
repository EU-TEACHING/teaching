import keras

from teaching.interface import TEACHINGNode
from .data import TEACHINGDataset


class LearningModule(TEACHINGNode):
    """LearningModule is the base class for all learning modules. It provides
    the skeleton exposing a keras.Model and a TEACHINGDataset. The default communication
    Interface is RabbitMQ
    """

    def __init__(
        self,
        produce_backend: str = "rabbitmq",
        consume_backend: str = "rabbitmq",
    ) -> None:
        super(LearningModule, self).__init__(produce_backend, consume_backend)
        self._model: keras.Model = None
        self._data: TEACHINGDataset = None

    def run(self):
        raise NotImplementedError

    @property
    def model(self):
        return self._model

    @model.setter
    def model(self, new_model):
        self._model = new_model
