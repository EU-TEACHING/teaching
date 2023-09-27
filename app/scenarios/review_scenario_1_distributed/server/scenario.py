from teaching.app.app import TEACHINGApplication
from teaching.ai_toolkit.training.federated.algorithms.fedavg import FedAvgServer

if __name__ == "__main__":
    app = TEACHINGApplication(volume="../../../storage")
    app.add_kafka(host="83.212.240.38")

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
