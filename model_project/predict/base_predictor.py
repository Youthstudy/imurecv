import numpy as np
from abc import ABC, abstractmethod


class BasePredictor(ABC):
    """
    所有预测器的统一接口
    """

    def __init__(self, feature_cols):
        """
        feature_cols: list[str]
        训练时使用的特征顺序
        """
        self.feature_cols = feature_cols

    def dict_to_array(self, feature_dict):
        """
        dict → np.array（严格按训练顺序）
        """
        return np.array(
            [feature_dict[c] for c in self.feature_cols],
            dtype=np.float32
        )

    @abstractmethod
    def predict(self, feature_dict):
        """
        输入一帧特征（dict）
        输出预测结果
        """
        pass
