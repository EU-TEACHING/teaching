from abc import ABC, abstractmethod
from modules.cybersecurity.src.utils.config import Config


class BaseInferrer(ABC):
    """Abstract Model class that is inherited to all models"""

    def __init__(self, cfg):
        self.config = Config.from_json(cfg)

    @abstractmethod
    def load_model(self):
        pass

    @abstractmethod
    def load_data(self):
        pass

    @abstractmethod
    def predict(self):
        pass
