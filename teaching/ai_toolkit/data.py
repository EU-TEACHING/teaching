from typing import Dict, List, Optional, Tuple, Union

import numpy as np


class TEACHINGDataset(object):
    def __init__(
        self,
        feature_keys: List[str],
        target_keys: Optional[List[str]] = None,
        min_size: int = 1000,
        max_size: int = 5000,
        incremental: bool = False,
    ) -> None:
        self.feature_keys = feature_keys
        self.target_keys = target_keys
        self.min_size = min_size
        self.max_size = max_size
        self.incremental = incremental

        self._data = []

    def prepare(
        self, msg_body: Dict, with_targets: bool = False
    ) -> Union[Tuple[np.ndarray, np.ndarray], np.ndarray]:
        x = np.array([msg_body[k] for k in self.feature_keys])
        if with_targets and self.target_keys is not None:
            y = np.array([msg_body[k] for k in self.target_keys])
            return x, y
        else:
            return x

    def __call__(
        self,
        msg_body: Dict,
        store: bool = True,
    ) -> None:
        if store:
            self._data.append(msg_body)
        if self.max_size > 0 and len(self) >= self.max_size:
            if self.incremental:
                self._data = self._data[1:]
            else:
                self._data = self._data[:-1]
        return self.prepare(msg_body, False)

    def __len__(self):
        return len(self._data)

    @property
    def samples(self) -> Tuple[np.ndarray, Optional[np.ndarray]]:
        if not self.ready:
            return None, None
        data = [
            self.prepare(msg_body, with_targets=self.target_keys is not None)
            for msg_body in self._data
        ]
        if self.target_keys is not None:
            x, y = zip(*data)
            x, y = np.stack(x), np.stack(y)
            # add dimension to position 0
            x, y = np.expand_dims(x, 1), np.expand_dims(y, 1)
            return x, y
        else:
            return np.expand_dims(np.stack(data), 1), None

    @property
    def ready(self) -> bool:
        return len(self) >= self.min_size
