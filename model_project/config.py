# config.py

CSV_PATH = r"model_project\data\all_features_with_gait_phase.csv"

# 自动检测到的特征组（你可通过变量控制是否启用）
USE_IMU = True
USE_DH = False
USE_FK = False
USE_JOINT = True     # 默认不使用真实膝角作为输入
USE_OTHER = False

# 三输出：phase, left_knee_angle, right_knee_angle
OUTPUT_COLS = ["gait_phase", "left_knee_angle", "right_knee_angle"]

# 未来预测参数
WINDOW = 50     # 输入序列长度
FUTURE = 5      # 预测未来多少帧

# 模型类型： "lstm", "cnn", "svm"
MODEL_TYPE = "svm"

# 训练参数
BATCH_SIZE = 64
EPOCHS = 5
LR = 1e-3

MODEL_SAVE_PATH = "model_project/result"
