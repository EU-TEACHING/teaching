from typing import Dict
import keras
import numpy as np

from teaching.ai_toolkit.models.esn import ESN


def serialize_model(model: keras.Model) -> Dict:
    """Encode a keras model into a dictionary with the config and weights.

    Args:
        model (keras.Model): The model to encode.

    Returns:
        Dict: The dictionary containing the model config and weights.
    """
    config = model.to_json()
    weights = [w.tolist() for w in model.get_weights()]
    return {"config": config, "weights": weights}


def deserialize_model(model_json: Dict) -> keras.Model:
    """Decode a keras model from a dictionary with the config and weights.

    Args:
        model_json (Dict): The dictionary containing the model config and weights.

    Returns:
        keras.Model: The decoded model.
    """
    try:
        model = keras.models.model_from_json(model_json["config"])
    except ValueError:
        model = keras.models.model_from_json(
            model_json["config"], custom_objects={"ESN": ESN}
        )
    model.set_weights([np.array(w) for w in model_json["weights"]])
    return model
