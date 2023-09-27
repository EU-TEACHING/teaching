import tensorflow as tf
from teaching.ai_toolkit.models.esn import ESN

if __name__ == "__main__":
    inputs = tf.keras.Input(batch_shape=(1, 1, 8))
    x = ESN(
        units=250,
        leaky=0.8,
        spectral_radius=0.9,
        return_sequences=True,
        stateful=True,
    )(inputs)
    outputs = tf.keras.layers.Dense(
        1,
        activation="sigmoid",
    )(x)
    model = tf.keras.Model(inputs=inputs, outputs=outputs, name="stress_model")
    model.summary()
    model.save("../../storage/models/stress")
