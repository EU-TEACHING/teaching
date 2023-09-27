import torch
import os
import numpy as np
from sklearn.model_selection import train_test_split
from torch.utils.data import TensorDataset


def load_wesad():
    DATA_DIR = "/raid/carta/DATA/WESAD/preprocessed"
    X_WES = torch.tensor(np.load(os.path.join(DATA_DIR, "x_train.npy"))).float()
    y_WES = torch.tensor(np.load(os.path.join(DATA_DIR, "y_train.npy"))).argmax(dim=1)

    # Splitting data into training(80%), validation(10%) and test-set(10%)
    (X_WES, X_testWES, y_WES, y_testWES) = train_test_split(
        X_WES, y_WES, test_size=0.1, train_size=0.9
    )
    (X_trainWES, X_valWES, y_trainWES, y_valWES) = train_test_split(
        X_WES, y_WES, test_size=0.1, train_size=0.9
    )

    train_data = TensorDataset(X_trainWES, y_trainWES)
    val_data = TensorDataset(X_valWES, y_valWES)
    test_data = TensorDataset(X_testWES, y_testWES)
    return train_data, val_data, test_data
