import os
import time
from ..packet import DataPacket

from .base import TEACHINGProducer, TEACHINGConsumer


class FileSystemProducer(TEACHINGProducer):
    """A producer that writes messages to the local file system. Note that this producer
    is able to exchange messages only with other nodes which are deployed within the
    same machine.
    """

    def __init__(self, path: str = os.environ["PRODUCE_PATH"]) -> None:
        self._out_dir = path
        os.makedirs(path, exist_ok=True)
        self._n_packet = {}

    def send(self, msg: DataPacket) -> None:
        if msg.service_name not in self._n_packet:
            self._n_packet[msg.service_name] = 0
        n = self._n_packet[msg.service_name]
        msg.to_file(os.path.join(self._out_dir, f"{msg.service_name}_{n}.dat"))
        self._n_packet[msg.service_name] += 1


class FileSystemConsumer(TEACHINGConsumer):
    """A consumer that reads messages from the local file system. Note that this consumer
    is able to exchange messages only with other nodes which are deployed within the
    same machine.
    """

    def __init__(self) -> None:
        super(FileSystemConsumer, self).__init__()
        self._paths = []

    def consume_loop(self):
        while True:
            for path in self._paths:
                new_files = os.listdir(path)
                if new_files != []:
                    for f in new_files:
                        f_path = os.path.join(path, f)
                        msg = readfile(f_path)
                        self._data.put(msg)
                        os.unlink(f_path)

    def subscribe(self, topic: str) -> None:
        self._paths.append(topic)


def readfile(f_path):
    success = False
    while not success:
        try:
            msg = DataPacket.from_file(f_path)
            success = True
        except:
            time.sleep(0.5)
    return msg
