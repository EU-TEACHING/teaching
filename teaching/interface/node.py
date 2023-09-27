from typing import Optional

from teaching.interface.communication.packet import DataPacket
from .communication import get_producer, get_consumer, CommunicationBackend


class TEACHINGNode(object):
    def __init__(
        self,
        produce_backend: CommunicationBackend,
        consume_backend: CommunicationBackend,
    ) -> None:
        self._produce_backend = produce_backend
        self._consume_backend = consume_backend
        self._producer = None
        self._consumer = None

    def _run(self):
        self.build()
        self.run()

    def run(self):
        raise NotImplementedError

    def build(self):
        raise NotImplementedError

    def send(self, msg: DataPacket):
        self.producer.send(msg)

    def receive(self, timeout: Optional[float] = None) -> Optional[DataPacket]:
        if not self.consumer._consume_thread.is_alive():
            self.consumer.start()
        return self.consumer.receive(timeout=timeout)

    def compile(self):
        attributes = {k: v for k, v in vars(self).items() if not k.startswith("_")}
        return attributes

    @property
    def producer(self):
        if self._producer is None and self._produce_backend is not None:
            self._producer = get_producer(self._produce_backend)
            print("Instantiated producer with backend: ", self._produce_backend)
        return self._producer

    @property
    def consumer(self):
        if self._consumer is None and self._consume_backend is not None:
            self._consumer = get_consumer(self._consume_backend)
            print("Instantiated consumer with backend: ", self._consume_backend)
        return self._consumer
