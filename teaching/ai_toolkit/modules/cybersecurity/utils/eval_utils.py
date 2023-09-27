import numpy as np
import pandas as pd
from scipy.spatial import distance
from sklearn.metrics import (
    accuracy_score,
    precision_score,
    recall_score,
    f1_score,
    confusion_matrix,
)

from modules.cybersecurity.src.utils.preprocessing_utils import (
    create_dataframe_of_predicted_labels,
)


def compute_mahalanobis_params(model, normal_data):
    """Compute the covariance matrix and the mean that are needed to compute the mahalanobis distance.
    This is done only for the normal data (norm_mahal_x_seq)"""
    reconstruction_error = model.predict(normal_data) - normal_data
    cov = np.cov(reconstruction_error.reshape(-1, normal_data.shape[-1]).T)
    mean = np.mean(reconstruction_error.reshape(-1, normal_data.shape[-1]), axis=0)
    return cov, mean


def evaluate_fbeta(threshold, normal_scores, anomaly_scores):
    beta = 0.5
    tp = np.array(anomaly_scores)[np.array(anomaly_scores) > threshold].size
    fp = len(anomaly_scores) - tp
    fn = np.array(normal_scores)[np.array(normal_scores) > threshold].size
    tn = len(normal_scores) - fn

    if tp == 0:
        return 0

    P = tp / (tp + fp)  # Precision
    R = tp / (tp + fn)  # Recall
    fbeta = (1 + beta * beta) * P * R / (beta * beta * P + R)
    return fbeta


def compute_threshold(normal_scores, anomaly_scores):
    upper = np.median(np.array(anomaly_scores))
    lower = np.median(np.array(normal_scores))
    scala = 20  # divide the range(min,max) into 20 parts, find the optimal threshold
    delta = (upper - lower) / scala
    candidate = lower
    threshold = 0
    result = 0

    for _ in range(scala):
        r = evaluate_fbeta(candidate, normal_scores, anomaly_scores)
        if r > result:
            result = r
            threshold = candidate
        candidate += delta
    return threshold


def compute_mahalanobis(reconstruction_error, mean, cov, time_steps):
    """Compute mahalanobis distance."""
    temp_reshape = reconstruction_error.reshape(-1, reconstruction_error.shape[-1])
    return np.mean(
        np.array(
            [
                distance.mahalanobis(mean, temp_reshape[i], cov)
                for i in range(len(temp_reshape))
            ]
        ).reshape(-1, time_steps),
        axis=1,
    )


def anomaly_scoring(model, data_seq, time_steps, cov, mean):
    """Compute anomaly scores, find anomalies and return the anomalous data indices and the threshold."""
    reconstruction_error = (
        model.predict(data_seq) - data_seq
    )  # same shape with data_seq i.e., (n_seq,timesteps,n_feat)
    anomaly_scores = compute_mahalanobis(
        reconstruction_error, mean, cov, time_steps
    )  # shape: (n_seq,)
    return anomaly_scores


def get_anomalies(model, data_seq, threshold, time_steps, cov, mean):
    """Create a list of the anomalous data indices."""
    anomaly_scores = anomaly_scoring(
        model, data_seq, time_steps, cov, mean
    )  # shape: (n_seq,)
    # Detect all the samples which are anomalies.
    anomalies = anomaly_scores > threshold  # boolean, shape (n_seq,)
    # data i is an anomaly if samples [(i - timesteps + 1) to (i)] are anomalies
    anomalous_data_indices = []
    # for data_idx in range(time_steps - 1, len(data_seq) - time_steps + 1):
    for data_idx in range(time_steps - 1, len(data_seq)):
        if np.all(anomalies[data_idx - time_steps + 1 : data_idx + 1]):
            anomalous_data_indices.append(data_idx)
    return anomalous_data_indices


def pred_eval(y_true, y_pred):
    """Calculate accuracy, precision, recall and f1 scores to evaluate the model."""
    accuracy = accuracy_score(y_true, y_pred)
    precision = precision_score(y_true, y_pred)
    recall = recall_score(y_true, y_pred)
    f1 = f1_score(y_true, y_pred)
    cm = confusion_matrix(y_true, y_pred)
    print("Confusion matrix:")
    print(cm)
    msg = "Accuracy %.2f, Precision %.2f, Recall %.2f, F1 %.2f" % (
        accuracy,
        precision,
        recall,
        f1,
    )
    print(msg)

    return accuracy, precision, recall, f1


def get_eval_metrics(model, data_x, data_y, threshold, time_steps, cov, mean):
    """Evaluate, i.e., compute the reconstruction error and compute mahalanobis to get the anomaly scores, filter them
    with the threshold and return the anomalous indices."""
    anomalous_data_indices = get_anomalies(
        model, data_x, threshold, time_steps, cov, mean
    )
    data_y_unseq = np.concatenate([data_y[:-1, 0], data_y[-1, :]])
    data_y_unseq = pd.DataFrame(data_y_unseq.reshape(-1, 1))
    y_pred = create_dataframe_of_predicted_labels(data_y_unseq, anomalous_data_indices)
    accuracy, precision, recall, f1 = pred_eval(
        data_y_unseq[time_steps:-time_steps], y_pred[time_steps:-time_steps]
    )
    return accuracy, precision, recall, f1
