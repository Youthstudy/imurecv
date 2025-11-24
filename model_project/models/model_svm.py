# models/model_svm.py

from sklearn.svm import SVR
import numpy as np

class SVMRegressor:
    def __init__(self):
        self.models = [SVR(), SVR(), SVR()]

    def train(self, X, Y):
        X = X.reshape(X.shape[0], -1)
        for i in range(3):
            self.models[i].fit(X, Y[:, i])

    def predict(self, X):
        X = X.reshape(X.shape[0], -1)
        preds = [m.predict(X) for m in self.models]
        return np.vstack(preds).T
