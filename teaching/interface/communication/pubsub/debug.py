import random

from .base import TEACHINGProducer, TEACHINGConsumer
from ..packet import DataPacket


class DebugProducer(TEACHINGProducer):
    """A producer that writes messages to the standard output."""

    def send(self, msg: DataPacket) -> None:
        print(f"Sending message: {msg}")


class DebugConsumer(TEACHINGConsumer):
    """A consumer that creates random DataPacket objects."""

    def consume_loop(self):
        while True:
            self._data.put(
                DataPacket(
                    service_name="debug_stream",
                    service_type="debug",
                    topic="debug",
                    body={"value": random.random()},
                )
            )

    def subscribe(self, topic: str) -> None:
        pass
