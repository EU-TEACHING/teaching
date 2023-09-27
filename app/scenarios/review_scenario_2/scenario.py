from teaching.app.app import TEACHINGApplication
from teaching.data.modules.influxdb import InfluxDBManager
from teaching.sensors.modules.file.csv_feed import CSVFeed
from custom_node.run import CustomModule


if __name__ == "__main__":
    app = TEACHINGApplication(volume="../../storage")
    app.add_rabbitmq()
    app.add_influxdb()
    data_manager = InfluxDBManager(["*.*.value"])
    user_service = CSVFeed(
        path="/storage/data/wesad/s2_mini.csv",
        output_topic="sensor.chest.value",
        transmit_rate=0.1,
    )
    custom_service = CustomModule(
        ["sensor.chest.value"],
        feature_keys=["acc_x", "acc_y", "acc_z", "ecg", "emg", "eda", "temp", "resp"],
    )
    app.add_node("influxdb_logger", data_manager, depends_on=["rabbitmq", "influxdb"])
    app.add_node(
        "user",
        user_service,
        depends_on=["influxdb_logger", "rabbitmq"],
    )
    app.add_node(
        "custom_predictor",
        custom_service,
        image="vdecaro/teaching-ai_toolkit:latest",
        depends_on=["user", "rabbitmq", "influxdb_logger"],
        custom_path="custom_node",
        custom_type="custom_node.CustomModule",
    )
    app.compile(".")
