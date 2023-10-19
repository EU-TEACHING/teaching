from teaching.app.app import TEACHINGApplication
from teaching.ai_toolkit.training.federated.algorithms.fedavg import FedAvgServer

if __name__ == "__main__":
    app = TEACHINGApplication(volume="../../../storage")
    app.add_kafka_broker(host_address="20.81.146.253")
    app.add_kafka_client()
    fed_server = FedAvgServer(
        topic="models.stress",
        model_path="/storage/models/stress",
        epochs=1,
        n_before_avg=2,
    )

    app.add_node(
        "fed_server",
        fed_server,
        depends_on=["kafka"],
    )
    app.compile(".")
