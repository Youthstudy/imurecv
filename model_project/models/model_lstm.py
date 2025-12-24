# models/model_lstm.py

import torch.nn as nn

class LSTMRegressor(nn.Module):
    def __init__(self, input_dim, hidden=128, layers=2, output_dim=3):
        super().__init__()
        self.lstm = nn.LSTM(input_dim, hidden, layers, dropout = 0.3, batch_first=True)
        self.fc = nn.Linear(hidden, output_dim)

    def forward(self, x):
        out, _ = self.lstm(x)
        last = out[:, -1]
        return self.fc(last)
