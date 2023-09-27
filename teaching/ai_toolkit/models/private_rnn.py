from ..rnn.rnn_lm import RecurrentNeuralNetworks
from utils.esn_lite.deep_rc import SimpleDeepESNClassifier

import tensorflow as tf
from tensorflow_privacy.privacy.analysis.rdp_accountant import compute_rdp
from tensorflow_privacy.privacy.analysis.rdp_accountant import get_privacy_spent
from tensorflow_privacy.privacy.optimizers.dp_optimizer_keras import DPKerasSGDOptimizer


class PrivateRNN(RecurrentNeuralNetworks):
    def __init__(
        self,
        microbatches=5,
        l2_norm_clip=1.0,
        noise_multiplier=0.1,
        **kwargs,  # parent arguments
    ):
        """Private RNN LM. See design docs for details.

        :param microbatches: Private-SGD hyperparameter. it must divide batch_size.
        :param l2_norm_clip: Clipping norm.
        :param noise_multiplier: Ratio of the standard deviation to `l2_norm_clip`.
            Larger values allow more privacy at the expense of the accuracy.
        :param kwargs:
        """
        # TODO: should be parent's arguments
        self.epochs = None
        self.model = None
        self.learning_rate = None
        self.batch_size = None
        super().__init__(**kwargs)

        # private-SGD hyperparameters
        self.microbatches = microbatches
        self.l2_norm_clip = l2_norm_clip
        self.noise_multiplier = noise_multiplier

    def train(self, train_data, test_data):
        X_train, y_train = train_data
        X_test, y_test = test_data

        # TODO: model's instantiation should be delegated to the parent class
        self.model = SimpleDeepESNClassifier(
            num_classes=4,
            input_scaling=1.8,
            leaky=0.4,
            spectral_radius=0.9,
            units=1000,
            layers=3,
        )

        optimizer = DPKerasSGDOptimizer(
            l2_norm_clip=self.l2_norm_clip,
            noise_multiplier=self.noise_multiplier,
            num_microbatches=self.microbatches,
            learning_rate=self.learning_rate,
        )

        # Need to compute per-example loss rather than average over a minibatch.
        loss = tf.keras.losses.CategoricalCrossentropy(
            from_logits=True, reduction=tf.losses.Reduction.NONE
        )

        self.model.compile(optimizer=optimizer, loss=loss, metrics=["accuracy"])
        self.model.fit(
            X_train,
            y_train,
            epochs=self.epochs,
            validation_data=(X_test, y_test),
            batch_size=self.batch_size,
            verbose=0,
        )

        res = self.model.evaluate(X_test, y_test, verbose=0)
        print(f"loss={res[0]:.2f}, acc={res[1]:.2f}")

        # privacy budget.
        eps, delta = self._compute_privacy_budget(
            self.epochs * X_train.shape[0] // self.batch_size, X_train.shape[0]
        )
        print(
            f"n_samples={X_train.shape[0]}, delta={delta:.6f}. Computed epsilon={eps:.2f}"
        )
        return delta, eps  # privacy budget

    def _compute_privacy_budget(self, steps, data_size):
        """Computes Epsilon/Delta values.

        :param steps: iteration steps
        :param data_size: number of samples
        :return:
        """
        if self.noise_multiplier == 0.0:
            return float("inf")
        orders = [1 + x / 10.0 for x in range(1, 100)] + list(range(12, 64))
        sampling_probability = self.batch_size / data_size
        rdp = compute_rdp(
            q=sampling_probability,
            noise_multiplier=self.noise_multiplier,
            steps=steps,
            orders=orders,
        )
        delta = 1 / data_size  # delta is fixed to 1 / data_size
        eps = get_privacy_spent(orders, rdp, target_delta=delta)[0]
        return eps, delta
