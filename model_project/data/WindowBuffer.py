from collections import deque
import numpy as np

class WindowBuffer:
    def __init__(self, window_size, feature_dim):
        self.window_size = window_size
        self.buf = deque(maxlen=window_size)
        self.feature_dim = feature_dim

    def push(self, feat):
        self.buf.append(feat)

    def ready(self):
        return len(self.buf) == self.window_size

    def get(self):
        return np.stack(self.buf, axis=0)
