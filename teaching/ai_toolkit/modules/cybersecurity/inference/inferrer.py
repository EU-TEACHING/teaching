# -*- coding: utf-8 -*-
"""Inferrer"""

# standard
import os
import sys
import joblib
import pandas as pd

# internal
from modules.cybersecurity.src.dataloader.dataloader import DataLoader
from modules.cybersecurity.src.utils.eval_utils import (
    get_anomalies,
    anomaly_scoring,
    get_eval_metrics,
)
from modules.cybersecurity.src.utils.preprocessing_utils import (
    create_sequences,
    create_dataframe_of_predicted_labels,
    transform_df,
)
from modules.cybersecurity.src.configs.config import CFG
from modules.cybersecurity.src.inference.base_inferrer import BaseInferrer

# external
os.environ["TF_CPP_MIN_LOG_LEVEL"] = "3"
from tensorflow import keras


class Inferrer(BaseInferrer):
    def __init__(self, config):
        super().__init__(config)

        self.model_path = self.config.inference.model_path
        self.transformer_path = self.config.inference.transformer_path
        self.ground_truth_cols = self.config.inference.ground_truth_cols
        self.data_types = self.config.data.data_types
        self.n_rows = self.config.data.n_rows
        self.features = self.config.data.features
        self.threshold = None

        self.data = None
        self.X = None
        self.y = None

        self.model = None
        self.transformer = None

        self.data_pred = None

    def load_model(self):
        """Load model and transformer in standalone mode."""
        if not os.path.exists(self.model_path):
            print("The model was not found")
            sys.exit()

        self.model = keras.models.load_model(self.model_path)
        self.transformer = self._load_transformer(self.transformer_path)

        return self.model

    def load_model_online(self, model_path, transformer_path):
        """Load model and transformer via the teaching-app."""
        self.model = keras.models.load_model(model_path)
        self.transformer = self._load_transformer(transformer_path)

        return self.model, self.transformer

    @staticmethod
    def _load_transformer(transformer_path):
        """Load transformer."""

        if not os.path.exists(transformer_path):
            print("The transformer was not found")
            sys.exit()
        else:
            transformer = joblib.load(transformer_path)

        return transformer

    def load_data(self):
        if self.ground_truth_cols:
            columns = self.features + self.ground_truth_cols
        else:
            columns = self.features
        dict_data_types = (
            dict(zip(self.features, self.data_types)) if self.data_types else None
        )
        self.data = DataLoader().load_data(
            columns, self.config.inference, dict_data_types, self.n_rows, "inference"
        )

        self.X = self.data.loc[:, self.features]
        if self.ground_truth_cols:
            self.y = self.data.loc[:, self.ground_truth_cols]

        self._preprocess()

    def load_data_online(self, data):
        if self.ground_truth_cols:
            columns = self.features + self.ground_truth_cols
        else:
            columns = self.features
        dict_data_types = (
            dict(zip(self.features, self.data_types)) if self.data_types else None
        )
        self.data = data

        self.X = self.data.loc[:, self.features]
        if self.ground_truth_cols:
            self.y = self.data.loc[:, self.ground_truth_cols]

        self._preprocess()

    def _preprocess(self):
        X_transformed = transform_df(self.X, self.transformer)
        self.X_transformed_seq = create_sequences(
            X_transformed, self.transformer.seq_time_steps
        )
        # If there is a ground truth column, sequence it
        if self.y is not None:
            self.y_transformed_seq = create_sequences(
                self.y, self.transformer.seq_time_steps
            )

    def predict(self):
        """Predicts results for the verification dataset."""

        anomalous_data_indices = get_anomalies(
            self.model,
            self.X_transformed_seq,
            self.transformer.threshold,
            self.transformer.seq_time_steps,
            self.transformer.mahalanobis_params["cov"],
            self.transformer.mahalanobis_params["mean"],
        )

        y_pred = create_dataframe_of_predicted_labels(self.X, anomalous_data_indices)
        self.data_pred = pd.concat([self.data, y_pred], axis=1)

        print(self.data_pred.head(10))
        return self.data_pred

    def eval(self):
        # If there is a ground truth column, evaluate predictions against it
        if self.ground_truth_cols is not None:
            # Evaluate against y ground truth (prints confusion matrix)
            a_accuracy, a_precision, a_recall, a_f1 = get_eval_metrics(
                self.model,
                self.X_transformed_seq,
                self.y_transformed_seq,
                self.transformer.threshold,
                self.transformer.seq_time_steps,
                self.transformer.mahalanobis_params["cov"],
                self.transformer.mahalanobis_params["mean"],
            )
        else:
            print(
                "Ground truth labels have not been defined for evaluation. If available, define it in the config file."
            )


if __name__ == "__main__":
    infer = Inferrer(CFG)
    infer.load_model()
    infer.load_data()
    res = infer.predict()
