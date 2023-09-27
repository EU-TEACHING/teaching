import os
import pandas as pd

from teaching.interface.communication import DataPacket
from ...node import LearningModule

from .inference.inferrer import Inferrer
from .configs.config import CFG


class CybersecurityModule(LearningModule):
    def __init__(
        self, model_id: str, transformer_id: str, model_path: str, seq_time_steps: int
    ):
        super(CybersecurityModule, self).__init__()
        self.model_id = model_id
        self.transformer_id = transformer_id
        self.model_path = model_path
        self.seq_time_steps = seq_time_steps

        self._infer = None

    def run(self):
        queue = []
        count = 0
        while True:
            msg = self.receive()
            print("count: " + str(count))
            queue.append(msg.body)
            if len(queue) >= self.seq_time_steps * 2 - 1:
                seq_df = pd.DataFrame(queue)
                self._infer.load_data_online(seq_df)
                pred_df = self._infer.predict()
                pred_list = pred_df.to_dict(orient="records")

                yield DataPacket(
                    topic="prediction.cybersecurity.value",
                    timestamp=msg.timestamp,
                    body=pred_list[len(pred_list) - self.seq_time_steps],
                )
                print(pred_list[len(pred_list) - 6])
                queue.pop(0)
            count += 1

    def build(self):
        self._infer = Inferrer(CFG)
        if self.model_path is not None and os.path.exists(self.model_path):
            self.model_path = os.path.join(self.model_path, self.model_id)
            transformer_path = os.path.join(self.model_path, self.transformer_id)

            self._model, self._transformer = self._infer.load_model_online(
                self.model_path, transformer_path
            )
            self._model.summary()
        else:
            print("Trained model was not found in self._self.model_path")
