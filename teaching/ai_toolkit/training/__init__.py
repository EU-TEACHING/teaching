from typing import Union
from teaching.ai_toolkit.training.federated.node import FederatedClient
from teaching.ai_toolkit.training.local import ModuleTrainer


def get_trainable(name: str) -> Union[FederatedClient, ModuleTrainer]:
    if name == "dummy":
        from .federated.algorithms.dummy import DummyClient as trainable

    elif name == "fedavg":
        from .federated.algorithms.fedavg import FedAvgClient as trainable

    elif name == "local":
        from local import KerasTrainer as trainable
    else:
        raise ValueError(f"Parameter {name} is invalid.")

    return trainable
