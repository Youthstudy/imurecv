import pandas as pd
import numpy as np
import torch
from torch.utils.data import Dataset, DataLoader


class FeatureDataset(Dataset):
    """
    加载 all_features.csv，生成滑动窗口数据
    支持多输入特征组合
    """
    def __init__(self, 
                 csv_path,
                 input_columns,
                 label_column=None,
                 window_size=50,
                 stride=1,
                 normalize=True):
        
        self.df = pd.read_csv(csv_path)
        self.input_columns = input_columns
        self.label_column = label_column
        self.window = window_size
        self.stride = stride
        self.normalize = normalize

        # 取输入特征矩阵
        self.X_raw = self.df[input_columns].to_numpy().astype(np.float32)

        # 标签（可选）
        if label_column:
            self.y_raw = self.df[label_column].to_numpy().astype(np.float32)
        else:
            self.y_raw = None

        # 标准化
        if normalize:
            self.mean = self.X_raw.mean(axis=0, keepdims=True)
            self.std = self.X_raw.std(axis=0, keepdims=True) + 1e-6
            self.X_raw = (self.X_raw - self.mean) / self.std
        else:
            self.mean = None
            self.std = None

        # 生成窗口索引
        self.indices = []
        T = len(self.X_raw)
        for start in range(0, T - window_size + 1, stride):
            end = start + window_size
            self.indices.append((start, end))


    def __len__(self):
        return len(self.indices)


    def __getitem__(self, idx):
        start, end = self.indices[idx]

        # X: shape [window, features]
        X = self.X_raw[start:end]

        # y: 使用窗口末尾的标签（常用于动作识别）
        if self.y_raw is not None:
            y = self.y_raw[end - 1]
            return torch.from_numpy(X), torch.tensor(y, dtype=torch.float32)

        return torch.from_numpy(X)
    

def create_dataloader(csv_path,
                      input_columns,
                      label_column=None,
                      window_size=50,
                      stride=1,
                      batch_size=64,
                      shuffle=True,
                      normalize=True,
                      num_workers=0):

    dataset = FeatureDataset(
        csv_path=csv_path,
        input_columns=input_columns,
        label_column=label_column,
        window_size=window_size,
        stride=stride,
        normalize=normalize
    )

    loader = DataLoader(
        dataset,
        batch_size=batch_size,
        shuffle=shuffle,
        num_workers=num_workers,
        drop_last=False
    )

    return loader


if __name__ == "__main__":
    print("Testing FeatureDataset and DataLoader...")
    csv_path = "all_features.csv"
    input_columns = ['pelvis_acc_x', 'pelvis_acc_y', 'pelvis_acc_z']
    label_column = 'pelvis_gyro_x'
    window_size = 50
    stride = 1
    batch_size = 32
    loader = create_dataloader(
        csv_path=csv_path,
        input_columns=input_columns,
        label_column=label_column,
        window_size=window_size,
        stride=stride,
        batch_size=batch_size,
        shuffle=False,
        normalize=False,
        num_workers=0
    )

    print(f"Total batches: {len(loader)}")
    for i, (X_batch, y_batch) in enumerate(loader):
        print(f"Batch {i}: X shape: {X_batch.shape}, y shape: {y_batch.shape}")
        if i == 0:  # 只打印前3个batch
            break