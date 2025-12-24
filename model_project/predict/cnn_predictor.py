import torch
import numpy as np
from collections import deque
from base_predictor import BasePredictor


class CNNPredictor(BasePredictor):
    """
    1D CNN 时序预测
    """

    def __init__(self,
                 model_path,
                 feature_cols,
                 window=50,
                 device="cpu"):
        super().__init__(feature_cols)

        self.window = window
        self.device = device

        self.model = torch.load(model_path, map_location=device)
        self.model.eval()

        self.buffer = deque(maxlen=window)

    def reset(self):
        self.buffer.clear()

    def predict(self, feature_dict):
        x = self.dict_to_array(feature_dict)
        self.buffer.append(x)

        if len(self.buffer) < self.window:
            return None

        # (1, C, T)
        seq = np.stack(self.buffer).T
        seq = torch.from_numpy(seq).unsqueeze(0)

        with torch.no_grad():
            y = self.model(seq.to(self.device))

        return y.cpu().numpy().squeeze()
