from teaching.interface import TEACHINGNode
from teaching.interface.communication import CommunicationBackend


class DataManager(TEACHINGNode):
    def __init__(
        self,
        produce_backend: CommunicationBackend = None,
        consume_backend: CommunicationBackend = "rabbitmq",
        **config
    ) -> None:
        super().__init__(produce_backend, consume_backend)

    def run(self):
        raise NotImplementedError
