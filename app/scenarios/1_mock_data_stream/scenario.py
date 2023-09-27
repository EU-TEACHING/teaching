import os
from teaching.app.app import TEACHINGApplication
from teaching.data.modules.influxdb import InfluxDBManager
from teaching.sensors.modules.file.csv_feed import CSVFeed

if __name__ == "__main__":
    app = TEACHINGApplication(volume="../../storage")
    app.add_rabbitmq()
    app.add_influxdb()
    data_manager = InfluxDBManager(["sensor.*.value"])
    carla_mock = CSVFeed(
        path="/storage/data/carla.csv",
        output_topic="sensor.carla.value",
        transmit_rate=0.1,
    )
    app.add_node("influxdb_logger", data_manager, depends_on=["rabbitmq", "influxdb"])
    app.add_node("carla_mock", carla_mock, depends_on=["influxdb_logger", "rabbitmq"])
    app.compile(".")
