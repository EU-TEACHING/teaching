import os
from typing import List

from urllib3.util.retry import Retry
from influxdb_client import InfluxDBClient, Point
from influxdb_client.client.write_api import SYNCHRONOUS

from ..node import DataManager


class InfluxDBManager(DataManager):
    def __init__(self, topics: List[str]):
        super().__init__(None, "rabbitmq")
        self.topics = topics
        self._bucket = None
        self._client = None
        self._write_api = None

    def run(self):
        while True:
            msg = self.receive()
            # print(msg)
            if isinstance(msg.timestamp, List):
                for i in range(len(msg.timestamp)):
                    p = Point.from_dict(
                        {
                            "measurement": msg.topic,
                            "fields": msg.body[i],
                            "time": msg.timestamp[i],
                        }
                    )
                    self._write_api.write(bucket=self._bucket, record=p)
            else:
                p = Point.from_dict(
                    {
                        "measurement": msg.topic,
                        "fields": msg.body,
                        "time": msg.timestamp,
                    }
                )
                self._write_api.write(bucket=self._bucket, record=p)

    def build(self):
        host = os.environ["INFLUXDB_HOST"]
        port = os.environ["INFLUXDB_PORT"]
        token = os.environ["INFLUXDB_TOKEN"]
        org = os.environ["INFLUXDB_ORG"]
        self._bucket = os.environ["INFLUXDB_BUCKET"]
        self._client = InfluxDBClient(
            url=f"http://{host}:{port}",
            token=token,
            org=org,
            retries=Retry(10, backoff_factor=2.0),
        )

        self._write_api = self._client.write_api(write_options=SYNCHRONOUS)
        for t in self.topics:
            self.consumer.subscribe(t)
