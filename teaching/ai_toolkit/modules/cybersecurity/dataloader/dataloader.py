# -*- coding: utf-8 -*-
"""Data Loader"""

import pandas as pd


class DataLoader:
    """Data Loader class"""

    @staticmethod
    def load_data(columns, config, data_types, n_rows, mode):
        """Loads dataset from path"""

        # mode: train or inference

        if mode == "train":
            # UNSW-NB15
            normal = pd.read_csv(
                config.path_normal, usecols=columns, dtype=data_types, index_col=False
            )
            anomaly = pd.read_csv(
                config.path_anomaly,
                usecols=columns,
                dtype=data_types,
                nrows=n_rows,
                na_values=" ",
                index_col=False,
            )
            anomaly.dropna(axis=0, inplace=True)
            anomaly.reset_index(inplace=True, drop=True)
            verification_set = pd.read_csv(
                config.verification_set,
                usecols=columns,
                dtype=data_types,
                nrows=n_rows,
                na_values=" ",
                index_col=False,
            )
            return normal, anomaly, verification_set

        elif mode == "inference":
            data = pd.read_csv(
                config.data_path,
                usecols=columns,
                dtype=data_types,
                nrows=n_rows,
                na_values=" ",
                index_col=False,
            )
            data.dropna(axis=0, inplace=True)
            data.reset_index(inplace=True, drop=True)
            return data
