# train/train_lstm.py

import torch
from torch.utils.data import DataLoader
from models.model_lstm import LSTMRegressor
from data.dataloader import FutureDataset
from config import BATCH_SIZE, EPOCHS, LR, MODEL_SAVE_PATH
from utils.model_saver import ModelSaver
import config

def train_lstm():
    ds = FutureDataset()
    dl = DataLoader(ds, batch_size=BATCH_SIZE, shuffle=True)

    input_dim = ds.X.shape[1]
    model = LSTMRegressor(input_dim=input_dim)
    model.train()
    saver = ModelSaver(save_dir= MODEL_SAVE_PATH)
    optim = torch.optim.Adam(model.parameters(), lr=LR)
    loss_fn = torch.nn.MSELoss()

    for epoch in range(EPOCHS):
        total = 0
        for X, y in dl:
            optim.zero_grad()
            pred = model(X)
            loss = loss_fn(pred, y)
            loss.backward()
            optim.step()
            total += loss.item()
        print(f"Epoch {epoch} Loss = {total:.4f}")
        saver.save_best(model, config=config, epoch=epoch, val_loss=total)
