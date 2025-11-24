import torch
import torch.nn as nn
import torch.nn.functional as F
import math
import pandas as pd
import numpy as np
from torch.utils.data import Dataset, DataLoader

# ===================== 1️⃣ 可微分DH层 =====================
class DHLayer(nn.Module):
    """
    可微分DH运动学层：根据角度预测末端坐标
    """
    def __init__(self, thigh_length=0.5, shank_length=0.45, hip_width=0.2):
        super().__init__()
        self.thigh_length = nn.Parameter(torch.tensor(thigh_length))
        self.shank_length = nn.Parameter(torch.tensor(shank_length))
        self.hip_width = hip_width

    def forward(self, hip_angle, knee_angle, side='left'):
        hip_offset = self.hip_width / 2 if side == 'left' else -self.hip_width / 2
        x = self.thigh_length * torch.cos(hip_angle) + \
            self.shank_length * torch.cos(hip_angle + knee_angle)
        z = -(self.thigh_length * torch.sin(hip_angle) + \
              self.shank_length * torch.sin(hip_angle + knee_angle))
        y = torch.ones_like(x) * hip_offset
        pos = torch.stack([x, y, z], dim=1)
        return pos


# ===================== 2️⃣ 主姿态估计网络 =====================
class PoseEstimator(nn.Module):
    """
    用IMU/编码器特征预测关节角，并通过DH层输出姿态坐标
    """
    def __init__(self, input_dim=8, hidden_dim=128):
        super().__init__()
        self.encoder = nn.Sequential(
            nn.Linear(input_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, hidden_dim // 2),
            nn.ReLU(),
        )
        self.angle_head = nn.Linear(hidden_dim // 2, 2)  # 输出：hip, knee
        self.dh_layer = DHLayer()

    def forward(self, x):
        feat = self.encoder(x)
        angles = self.angle_head(feat)
        hip = angles[:, 0]
        knee = angles[:, 1]
        pos = self.dh_layer(hip, knee)
        return pos, angles


# ===================== 3️⃣ 数据集包装 =====================
class PoseDataset(Dataset):
    """
    从CSV中读取IMU+编码器数据，并给出真值角度或足端位置
    """
    def __init__(self, csv_path):
        df = pd.read_csv(csv_path)
        # 示例: 取 IMU 加速度+角速度+编码器角度
        features = ['Acc_X_1', 'Acc_Y_1', 'Acc_Z_1',
                    'Gyro_X_1', 'Gyro_Y_1', 'Gyro_Z_1',
                    'ret0_joint1', 'ret0_joint2', ]
        labels = ['label']
        self.X = torch.tensor(df[features].values, dtype=torch.float32)
        self.Y = torch.tensor(df[labels].values, dtype=torch.float32)

    def __len__(self):
        return len(self.X)

    def __getitem__(self, idx):
        return self.X[idx], self.Y[idx]


# ===================== 4️⃣ 训练逻辑 =====================
def train_pose_estimator(csv_path, epochs=10, lr=1e-3, batch_size=64):
    dataset = PoseDataset(csv_path)
    dataloader = DataLoader(dataset, batch_size=batch_size, shuffle=True)
    model = PoseEstimator(input_dim=8)
    optimizer = torch.optim.Adam(model.parameters(), lr=lr)
    loss_fn = nn.MSELoss()

    for epoch in range(epochs):
        total_loss = 0
        for X, Y in dataloader:
            pred_pos, pred_angles = model(X)
            # 使用末端坐标真值 (Y[:,2:]) 进行监督
            loss = loss_fn(pred_pos, Y[:, 2:])
            optimizer.zero_grad()
            loss.backward()
            optimizer.step()
            total_loss += loss.item() * len(X)
        print(f"Epoch {epoch+1}/{epochs}, Loss = {total_loss/len(dataset):.6f}")

    torch.save(model.state_dict(), "pose_estimator_dh.pth")
    print("✅ 训练完成，模型已保存。")
    return model


# ===================== 5️⃣ 测试 =====================
if __name__ == "__main__":
    csv_path = r"F:\3.biye\1_code\imurecv\data\merged\merged_20251106_183706.csv"
    model = train_pose_estimator(csv_path, epochs=20)
