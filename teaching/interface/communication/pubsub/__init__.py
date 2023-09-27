import os
from typing import Literal
import typing
from .base import TEACHINGProducer, TEACHINGConsumer

CommunicationBackend = Literal["rabbitmq", "kafka", "fs", "debug"]


def get_producer(
    backend: CommunicationBackend = os.getenv("PRODUCER", "debug"), **producer_args
) -> TEACHINGProducer:
    """Get a producer for the specified backend.

    Args:
        backend (Literal["rabbitmq", "kafka", "fs"]): The backend to use.
        **producer_args: The arguments to pass to the producer.

    Returns:
        TEACHINGProducer: The producer.
    """
    if backend == "rabbitmq":
        from .rabbitmq import RabbitMQProducer

        return RabbitMQProducer(**producer_args)
    elif backend == "kafka":
        from .kafka import KafkaProducer

        return KafkaProducer(**producer_args)
    elif backend == "fs":
        from .file_system import FileSystemProducer

        return FileSystemProducer(**producer_args)
    elif backend == "debug":
        from .debug import DebugProducer

        return DebugProducer(**producer_args)
    else:
        raise NotImplementedError("Alternative producer backends need implementation.")


def get_consumer(
    backend: Literal["rabbitmq", "kafka", "fs"] = os.getenv("CONSUMER", "debug"),
    **consumer_args
) -> TEACHINGConsumer:
    """Get a consumer for the specified backend.

    Args:
        backend (Literal["rabbitmq", "kafka", "fs"]): The backend to use.
        **consumer_args: The arguments to pass to the consumer.

    Returns:
        TEACHINGConsumer: The consumer.
    """
    if backend == "rabbitmq":
        from .rabbitmq import RabbitMQConsumer

        return RabbitMQConsumer(**consumer_args)
    elif backend == "kafka":
        from .kafka import KafkaConsumer

        return KafkaConsumer(**consumer_args)
    elif backend == "fs":
        from .file_system import FileSystemConsumer

        return FileSystemConsumer(**consumer_args)
    elif backend == "debug":
        from .debug import DebugConsumer

        return DebugConsumer(**consumer_args)
    else:
        raise NotImplementedError("Alternative consumer backends need implementation.")
