from teaching.app.app import TEACHINGApplication
from teaching.data.modules.influxdb import InfluxDBManager
from teaching.sensors.modules.file.csv_feed import CSVFeed
from teaching.ai_toolkit.modules.stress_prediction import StressModule

if __name__ == "__main__":
    app = TEACHINGApplication(volume="../../storage")
    app.add_rabbitmq()
    app.add_influxdb()
    data_manager = InfluxDBManager(["*.*.value"])
    eda_mock = CSVFeed(
        path="/storage/data/wesad.csv",
        output_topic="sensor.eda.value",
        transmit_rate=0.1,
    )
    stress_service = StressModule(["sensor.eda.value"])
    app.add_node("influxdb_logger", data_manager, depends_on=["rabbitmq", "influxdb"])
    app.add_node("eda_mock", eda_mock, depends_on=["influxdb_logger", "rabbitmq"])
    app.add_node(
        "stress_service",
        stress_service,
        depends_on=["eda_mock", "rabbitmq", "influxdb_logger"],
    )
    app.compile(".")
