import numpy as np
import pandas as pd

from sklearn.preprocessing import MinMaxScaler
from sklearn.pipeline import Pipeline
from sklearn.preprocessing import FunctionTransformer
from sklearn.compose import ColumnTransformer
from sklearn.preprocessing import OneHotEncoder


def features_to_float(df):
    """Turn all columns of the dataframe to type float32 and reset index."""
    df = df.astype("float32")
    df = df.reset_index(drop=True)
    return df


def create_sequences(values, time_steps):
    """Generated training sequences for use in the model."""
    output = []
    for i in range(len(values) - time_steps + 1):
        output.append(values[i : (i + time_steps)])
    return np.stack(output)


def fill_na(df, numeric_columns, method="interp"):
    """Impute missing values. First row with backward fill, last row with forward fill, intermediate rows with pchip
    interpolation."""
    df_original = df.copy()
    df = df.loc[:, numeric_columns]
    # Backward fill for the first row
    df_bfill = df.fillna(method="bfill")
    # Forward fill for the last row
    df_ffill = df.fillna(method="ffill")

    # Replace filled first and last row
    df.iloc[0, :] = df_bfill.iloc[0, :]
    df.iloc[-1, :] = df_ffill.iloc[-1, :]

    if method == "interp":
        # Pchip interpolation for intermediate rows
        df_interp = df.interpolate(method="pchip")

        print(f"Missing values: {df_interp.isna().any().sum()}")
        print(f"Negative interpolated values: {df_interp.lt(0).sum().sum()}")

        if df_interp.lt(0).sum().sum() > 0:
            print("!Indices of negative values after interpolation with pchip!")
            print(f"Columns: {df_interp.columns[df_interp.lt(0).any(axis=0)]}")
            print(f"Rows: {df_interp[df_interp.lt(0).any(axis=1)].index.tolist()}")

        df_original.loc[:, numeric_columns] = df_interp

    return df_original


def get_numeric_features(df):
    """Create a list of the numeric features."""
    numerics = ["int16", "int32", "int64", "float16", "float32", "float64"]
    numeric_features = df.select_dtypes(include=numerics).columns.tolist()
    return numeric_features


def get_categorical_features(df):
    """Create a list of the categorical features."""
    categories = ["category", "object", "bool"]
    categorical_features = df.select_dtypes(include=categories).columns.tolist()
    return categorical_features


def transformed_data_to_df(transformed_data):
    """Turn transformed_data to dataframe."""
    if isinstance(transformed_data, np.ndarray):
        df = pd.DataFrame(transformed_data)
    else:
        array = transformed_data.toarray()
        df = pd.DataFrame(array)
    return df


def dtype_to_category(df, categorical_features):
    """Change the dtypes of specific features to category."""
    df[categorical_features] = df[categorical_features].astype("category")
    return df


def main_transformer(df):
    """Forward the df argument with ColumnTransformer to MinMaxScaler and OneHotEncoder , then with FunctionTransformer
    to the user-defined function transformed_to_df and return the results. Construct a pipeline with these results.
    Fit the pipeline (transformer) on the df. Return the pipeline."""
    numeric_features = get_numeric_features(df)
    categorical_features = get_categorical_features(df)

    t = [
        ("num", MinMaxScaler(feature_range=(0, 1)), numeric_features),
        ("cat", OneHotEncoder(handle_unknown="ignore"), categorical_features),
    ]
    column_transformer = ColumnTransformer(
        transformers=t, n_jobs=-1, remainder="passthrough"
    )
    transformed_to_df = FunctionTransformer(transformed_data_to_df, validate=False)
    # Create the pipeline
    pipe = Pipeline(
        memory=None,
        steps=[
            ("col_transformer", column_transformer),
            ("transformed_to_df", transformed_to_df),
        ],
        verbose=True,
    )

    pipe.fit(df)
    pipe.total_features = (
        pipe.named_steps["col_transformer"]
        .transformers_[0][1]
        .get_feature_names_out()
        .tolist()
        + pipe.named_steps["col_transformer"]
        .transformers_[1][1]
        .get_feature_names_out(categorical_features)
        .tolist()
    )
    return pipe


def change_column_names(df, total_features):
    """Rename the column names of the dataframe."""
    df.columns = total_features
    # Reposition the label column to the end of the df
    if "label" in df.columns:
        label = df.pop("label")
        df = df.assign(label=label)

    return df


def create_dataframe_of_predicted_labels(original_y, anomalous_data_indices):
    """Create a dataframe of zeros and add 1s at the positions indicated by the anomalous_data_indices. Return a
    dataframe of the predicted labels of the anomaly dataset."""
    zeros = np.zeros(shape=(len(original_y), 1), dtype=int)
    anomaly_y_pred = pd.DataFrame(zeros, columns=["pred_label"])
    anomaly_y_pred.loc[anomalous_data_indices, "pred_label"] = 1
    return anomaly_y_pred


def transform_df(df, transformer):
    """Transform the data, rename the column names and change the dtypes to float32."""
    # Transform the data
    df = transformer.transform(df)
    # Change the names of the columns
    df = change_column_names(df, transformer.total_features)
    # Turn all the features to float
    df = features_to_float(df)
    return df
