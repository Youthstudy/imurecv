# models/model_cnn.py

import torch.nn as nn

class CNNRegressor(nn.Module):
    def __init__(self, input_dim, output_dim=3):
        super().__init__()

        self.net = nn.Sequential(
            nn.Conv1d(input_dim, 64, kernel_size=5, padding=2),
            nn.ReLU(),
            nn.Conv1d(64, 128, kernel_size=5, padding=2),
            nn.ReLU(),
            nn.AdaptiveAvgPool1d(1),
        )

        self.fc = nn.Linear(128, output_dim)

    def forward(self, x):
        x = x.transpose(1, 2)
        f = self.net(x).squeeze(-1)
        return self.fc(f)
