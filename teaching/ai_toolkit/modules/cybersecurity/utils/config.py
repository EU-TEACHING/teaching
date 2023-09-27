# -*- coding: utf-8 -*-
"""Config class"""

import json


class Config:
    """Config class which contains data, train and model hyperparameters"""

    def __init__(
        self, data, tune, train, model, anomaly_scoring, mlflow_config, inference
    ):
        self.data = data
        self.tune = tune
        self.train = train
        self.model = model
        self.anomaly_scoring = anomaly_scoring
        self.mlflow_config = mlflow_config
        self.inference = inference

    @classmethod
    def from_json(cls, cfg):
        """Creates config from json"""
        params = json.loads(json.dumps(cfg), object_hook=HelperObject)
        return cls(
            params.data,
            params.tune,
            params.train,
            params.model,
            params.anomaly_scoring,
            params.mlflow_config,
            params.inference,
        )


class HelperObject(object):
    """Helper class to convert json into Python object"""

    def __init__(self, dict_):
        self.__dict__.update(dict_)
