import os
from teaching.app.app import TEACHINGApplication
from teaching.data.modules.influxdb import InfluxDBManager
from teaching.sensors.modules.file.csv_feed import CSVFeed
from teaching.ai_toolkit.modules.stress_prediction import StressModule

if __name__ == "__main__":
    user_id = int(os.environ["USER_ID"])
    if not user_id in [2, 3]:
        raise ValueError("User ID must be 2 or 3")
    app = TEACHINGApplication(volume="../../../storage")
    app.add_rabbitmq()
    app.add_influxdb()
    app.add_kafka_client(host="20.81.146.253", port=29092)
    data_manager = InfluxDBManager(
        ["sensor.chest.value.*", "prediction.stress.value.*"]
    )
    user = CSVFeed(
        path=f"/storage/data/wesad/s{user_id}_mini.csv",
        output_topic="sensor.chest.value",
        transmit_rate=0.01,
    )

    stress_service = StressModule(
        ["sensor.chest.value"],
        feature_keys=["acc_x", "acc_y", "acc_z", "ecg", "emg", "eda", "temp", "resp"],
        target_keys=["label"],
        model_path="/storage/models/stress",
        output_topic="prediction.stress.value",
        trainable_args={
            "type": "fedavg",
            "topic": "models.stress",
            "epochs": 4,
            "loss": "binary_crossentropy",
        },
    )

    app.add_node("influxdb_logger", data_manager, depends_on=["rabbitmq", "influxdb"])
    app.add_node("user", user, depends_on=["influxdb_logger", "rabbitmq"])
    app.add_node(
        "stress_service",
        stress_service,
        depends_on=["user", "rabbitmq", "influxdb_logger"],
    )
    app.compile(".")
