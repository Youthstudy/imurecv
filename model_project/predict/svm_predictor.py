import joblib
import numpy as np
from base_predictor import BasePredictor


class SVMPredictor(BasePredictor):
    """
    单帧 SVM 预测（无时间依赖）
    """

    def __init__(self,
                 model_path,
                 feature_cols):
        super().__init__(feature_cols)

        self.model = joblib.load(model_path)

    def predict(self, feature_dict):
        x = self.dict_to_array(feature_dict)
        x = x.reshape(1, -1)

        y = self.model.predict(x)

        return y[0]
