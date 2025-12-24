import threading
import time
import queue


class InferenceThread(threading.Thread):
    def __init__(self,
                 predictor,
                 feature_provider,
                 result_queue: queue.Queue,
                 period=0.001):
        """
        predictor: LSTM / CNN / SVM predictor
        feature_provider: callable -> feature_dict
        result_queue: 线程安全队列
        period: 推理周期（秒）
        """
        super().__init__(daemon=True)

        self.predictor = predictor
        self.feature_provider = feature_provider
        self.result_queue = result_queue
        self.period = period

        self._running = threading.Event()
        self._running.set()

    def stop(self):
        self._running.clear()

    def run(self):
        while self._running.is_set():
            t0 = time.time()
            try:
                feature = self.feature_provider()
                if feature is not None:
                    y = self.predictor.predict(feature)
                    if y is not None:
                        self.result_queue.put(y)
            except Exception as e:
                self.result_queue.put(("error", str(e)))

            # 保持周期
            dt = time.time() - t0
            sleep_t = self.period - dt
            if sleep_t > 0:
                time.sleep(sleep_t)
