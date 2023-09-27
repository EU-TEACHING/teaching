from teaching.interface import TEACHINGNode
from teaching.interface.communication import CommunicationBackend


class SensorModule(TEACHINGNode):
    def __init__(
        self,
        produce_backend: CommunicationBackend = "rabbitmq",
        consume_backend: CommunicationBackend = None,
    ) -> None:
        super().__init__(produce_backend, consume_backend)

    def run(self):
        raise NotImplementedError
