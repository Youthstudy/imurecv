# data/dataloader.py

import pandas as pd
import numpy as np
import torch
from torch.utils.data import Dataset

from .feature_detect import detect_feature_groups
from config import *

class FutureDataset(Dataset):
    def __init__(self, normalize=True, eps=1e-6):
        if(MODEL_MODE == "train"):
            df = pd.read_csv(CSV_PATH)
            self.df = df
        # 自动识别列
            groups = detect_feature_groups(df.columns)

        # 拼接输入
        self.input_cols = []
        if USE_IMU: self.input_cols += groups["imu"]
        if USE_DH:  self.input_cols += groups["dh"]
        if USE_FK:  self.input_cols += groups["fk"]
        if USE_JOINT: self.input_cols += groups["joint"]
        if USE_OTHER: self.input_cols += groups["other"]

        # 输入矩阵
        X = df[self.input_cols].values.astype(np.float32)

        # ====== 归一化处理 ======
        self.normalize = normalize
        self.eps = eps

        if self.normalize:
            self.mean = X.mean(axis=0)
            self.std = X.std(axis=0)
            self.std[self.std < eps] = 1.0   # 防止除 0

            X = (X - self.mean) / self.std
        else:
            self.mean = None
            self.std = None

        self.X = X

        # 输出（3 个）
        self.Y = df[OUTPUT_COLS].values.astype(np.float32)

        T = len(self.X)
        self.window = WINDOW
        self.future = FUTURE

        self.indices = []
        for i in range(T - self.window - self.future):
            self.indices.append((i, i + self.window, i + self.window + self.future - 1))

    def __len__(self):
        return len(self.indices)

    def __getitem__(self, idx):
        s, e, f = self.indices[idx]
        X = self.X[s:e]
        y = self.Y[f]
        return torch.tensor(X), torch.tensor(y)
