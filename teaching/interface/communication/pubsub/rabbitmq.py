import os
from threading import Thread
import pika

from ..packet import DataPacket
from .base import TEACHINGProducer, TEACHINGConsumer


class RabbitMQProducer(TEACHINGProducer):
    def __init__(
        self,
        user: str = os.getenv("RABBITMQ_USER", "teaching"),
        password: str = os.getenv("RABBITMQ_PASSWORD", "asd123456"),
        host: str = os.getenv("RABBITMQ_HOST", "rabbitmq"),
        port: str = os.getenv("RABBITMQ_PORT", "5672"),
    ):
        self._config = pika.ConnectionParameters(
            host=host,
            port=port,
            virtual_host="/",
            credentials=pika.PlainCredentials(user, password),
            connection_attempts=15,
            retry_delay=10,
            socket_timeout=5,
            heartbeat=600,
        )
        self._connection = pika.BlockingConnection(self._config)
        self._channel = self._connection.channel()

    def send(self, msg: DataPacket) -> None:
        self._channel.basic_publish(
            exchange="amq.topic", routing_key=msg.topic, body=msg.dumps()
        )


class RabbitMQConsumer(TEACHINGConsumer):
    def __init__(
        self,
        user: str = os.getenv("RABBITMQ_USER", "teaching"),
        password: str = os.getenv("RABBITMQ_PASSWORD", "asd123456"),
        host: str = os.getenv("RABBITMQ_HOST", "rabbitmq"),
        port: str = os.getenv("RABBITMQ_PORT", "5672"),
    ):
        super(RabbitMQConsumer, self).__init__()
        self._config = pika.ConnectionParameters(
            host=host,
            port=port,
            virtual_host="/",
            credentials=pika.PlainCredentials(user, password),
            connection_attempts=15,
            retry_delay=10,
            socket_timeout=5,
            heartbeat=600,
        )
        self._connection = pika.BlockingConnection(self._config)
        self._channel = self._connection.channel()

        self._queue = f"{os.environ['SERVICE_NAME']}.queue"
        self._channel.queue_declare(queue=self._queue, exclusive=True, auto_delete=True)

    def start(self):
        self._channel.basic_consume(
            self._queue,
            on_message_callback=lambda ch, method, properties, body: self._data.put(
                DataPacket.from_json(body)
            ),
            auto_ack=True,
        )
        self._consume_thread = Thread(target=self._channel.start_consuming)
        self._consume_thread.start()

    def subscribe(self, topic: str) -> None:
        self._channel.queue_bind(
            exchange=f"amq.topic", queue=self._queue, routing_key=topic
        )
