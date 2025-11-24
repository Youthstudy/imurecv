# train/train_svm.py

import numpy as np
from data.dataloader import FutureDataset
from models.model_svm import SVMRegressor
from config import BATCH_SIZE, EPOCHS, LR, MODEL_SAVE_PATH
from utils.model_saver import ModelSaver
import config

def train_svm():
    ds = FutureDataset()
    saver = ModelSaver(save_dir= MODEL_SAVE_PATH)
    X_list, Y_list = [], []
    for X, y in ds:
        X_list.append(X.numpy())
        Y_list.append(y.numpy())

    X = np.stack(X_list)
    Y = np.stack(Y_list)

    model = SVMRegressor()
    model.train(X, Y)
    saver.save_best(model, config=config, epoch=EPOCHS, val_loss=0)  # SVM 没有 epoch 和 loss，这里随便传
    print("SVM model saved.")
