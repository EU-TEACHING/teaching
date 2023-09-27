from queue import Empty, Queue
import threading
from typing import Optional
from ..packet import DataPacket


class TEACHINGProducer:
    """Base class for all TEACHING producers. Producers are used to send messages to
    other nodes within a TEACHING application.
    """

    def send(self, msg: DataPacket) -> bool:
        """
        Sends a message to the other nodes within a TEACHING application.
        """
        raise NotImplementedError


class TEACHINGConsumer:
    """Base class for all TEACHING consumers. Consumers are used to receive messages
    from other nodes within a TEACHING application.
    """

    def __init__(self) -> None:
        self._data = Queue()
        self._consume_thread = threading.Thread(target=self.consume_loop)

    def consume_loop(self) -> None:
        """
        The main loop of the consumer.
        """
        raise NotImplementedError

    def start(self) -> None:
        """
        Starts the consume loop.
        """
        self._consume_thread.start()

    def receive(self, timeout: Optional[float]) -> Optional[DataPacket]:
        """
        Receives a message from the other nodes within a TEACHING application.

        Args:
            timeout (Optional[float]): The maximum time to wait for a message. If
                None, wait indefinitely.

        Returns:
            Optional[DataPacket]: The received message, or None if no message was
                received. Note that the message may be None only if the timeout
                parameter is set.
        """
        try:
            msg = self._data.get(timeout=timeout)
        except Empty:
            msg = None
        return msg

    def subscribe(self, topic: str) -> None:
        """
        Subscribes to a topic.

        Args:
            topic (str): The topic to subscribe to.
        """
        raise NotImplementedError
