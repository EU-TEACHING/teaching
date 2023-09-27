import csv
import time

from teaching.interface.communication import DataPacket
from ...node import SensorModule


class CSVFeed(SensorModule):
    def __init__(
        self,
        path: str,
        output_topic: str,
        transmit_rate: float,
        close_at_end: bool = True,
    ) -> None:
        super().__init__()
        self.path = path
        self.output_topic = output_topic
        self.transmit_rate = transmit_rate
        self.close_at_end = close_at_end
        self._rows = []
        self._headers = []

    def run(self):
        i = 0
        while True:
            msg = DataPacket(
                topic=self.output_topic,
                body=dict(zip(self._headers, self._rows[i])),
            )
            # print(msg)
            self.send(msg)

            i = i + 1 if i < len(self._rows) - 1 else 0
            time.sleep(self.transmit_rate)
            if i == 0 and self.close_at_end:
                print("Finished sending all the data!", flush=True)
                break

    def build(self):
        print("Building the CSV file reader...")
        with open(self.path) as csv_file:
            csv_reader = csv.reader(csv_file, delimiter=",")
            self._headers = next(csv_reader)
            for row in csv_reader:
                self._rows.append([float(x) for x in row])
        print("Done!")
