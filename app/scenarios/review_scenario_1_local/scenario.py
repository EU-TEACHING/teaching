from teaching.app.app import TEACHINGApplication
from teaching.data.modules.influxdb import InfluxDBManager
from teaching.sensors.modules.file.csv_feed import CSVFeed
from teaching.ai_toolkit.modules.stress_prediction import StressModule
from teaching.ai_toolkit.training.federated.algorithms.fedavg import FedAvgServer

if __name__ == "__main__":
    app = TEACHINGApplication(volume="../../storage")
    app.add_rabbitmq()
    app.add_influxdb()
    app.add_kafka_broker()
    app.add_kafka_client()
    data_manager = InfluxDBManager(
        ["sensor.chest.value.*", "prediction.stress.value.*"]
    )
    user_2 = CSVFeed(
        path="/storage/data/wesad/s2_mini.csv",
        output_topic="sensor.chest.value.2",
        transmit_rate=0.01,
    )
    user_3 = CSVFeed(
        path="/storage/data/wesad/s3_mini.csv",
        output_topic="sensor.chest.value.3",
        transmit_rate=0.01,
    )

    stress_service_2 = StressModule(
        ["sensor.chest.value.2"],
        feature_keys=["acc_x", "acc_y", "acc_z", "ecg", "emg", "eda", "temp", "resp"],
        target_keys=["label"],
        model_path="/storage/models/stress",
        output_topic="prediction.stress.value.2",
        trainable_args={
            "type": "fedavg",
            "topic": "models.stress",
            "epochs": 4,
            "loss": "binary_crossentropy",
        },
    )
    stress_service_3 = StressModule(
        ["sensor.chest.value.3"],
        feature_keys=["acc_x", "acc_y", "acc_z", "ecg", "emg", "eda", "temp", "resp"],
        target_keys=["label"],
        model_path="/storage/models/stress",
        output_topic="prediction.stress.value.3",
        trainable_args={
            "type": "fedavg",
            "topic": "models.stress",
            "epochs": 4,
            "loss": "binary_crossentropy",
        },
    )
    fed_server = FedAvgServer(
        topic="models.stress",
        model_path="/storage/models/stress",
        epochs=1,
        n_before_avg=2,
    )
    app.add_node("influxdb_logger", data_manager, depends_on=["rabbitmq", "influxdb"])
    app.add_node("user_2", user_2, depends_on=["influxdb_logger", "rabbitmq"])
    app.add_node("user_3", user_3, depends_on=["influxdb_logger", "rabbitmq"])
    app.add_node(
        "stress_service_2",
        stress_service_2,
        depends_on=["user_2", "rabbitmq", "influxdb_logger"],
    )
    app.add_node(
        "stress_service_3",
        stress_service_3,
        depends_on=["user_3", "rabbitmq", "influxdb_logger"],
    )
    app.add_node(
        "fed_server",
        fed_server,
        depends_on=["stress_service_2", "stress_service_3", "rabbitmq", "kafka"],
    )
    app.compile(".")
