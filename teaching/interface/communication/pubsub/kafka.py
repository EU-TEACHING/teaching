import uuid
import socket, os
import logging
from confluent_kafka import Consumer, Producer
from confluent_kafka import KafkaError, KafkaException

from ..packet import DataPacket
from .base import TEACHINGProducer, TEACHINGConsumer


class KafkaProducer(TEACHINGProducer):
    def __init__(
        self,
        broker_address: str = f"{os.getenv('KAFKA_HOST')}:{os.getenv('KAFKA_PORT')}",
        client_id: str = f"{socket.gethostname()}.{os.getenv('SERVICE_NAME')}",
    ) -> None:
        self._config = {"bootstrap.servers": broker_address, "client.id": client_id}
        self.producer = Producer(**self._config)

    def send(self, msg: DataPacket) -> None:
        self.producer.produce(msg.topic, value=msg.dumps())
        self.producer.flush()


class KafkaConsumer(TEACHINGConsumer):
    def __init__(
        self,
        broker_address: str = f"{os.getenv('KAFKA_HOST')}:{os.getenv('KAFKA_PORT')}",
        groupid: str = os.getenv("GROUPID", uuid.uuid4().hex),
        client_id: str = f"{socket.gethostname()}.{os.getenv('SERVICE_NAME')}",
        poll_timeout: float = float(os.getenv("POLL_TIMEOUT", "0.5")),
    ):
        super(KafkaConsumer, self).__init__()
        self._config = {
            "bootstrap.servers": broker_address,
            "client.id": client_id,
            "group.id": groupid,
            "auto.offset.reset": "smallest",
        }
        self._timeout = poll_timeout
        self._topics = []

        self.consumer = Consumer(**self._config)

    def consume_loop(self):
        while True:
            msg = self.consumer.poll(self._timeout)
            if msg is not None and msg.error():
                if msg.error().code() == KafkaError._PARTITION_EOF:
                    # End of partition event
                    logging.error(
                        "%% %s [%d] end at offset %d\n"
                        % (msg.topic(), msg.partition(), msg.offset())
                    )
                elif msg.error():
                    raise KafkaException(msg.error())
            else:
                msg = DataPacket.from_json(msg.value()) if msg is not None else None
                if msg is not None and not "dummy" in msg.body:
                    self._data.put(msg)

    def subscribe(self, topic: str) -> None:
        producer = Producer(**self._config)
        producer.produce(
            topic, value=DataPacket(topic=topic, body={"dummy": True}).dumps()
        )
        producer.flush()
        self.consumer.subscribe([topic])
