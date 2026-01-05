import os
import torch
import json
import joblib
from datetime import datetime

class ModelSaver:
    """
    训练过程中只保留一个最优模型，同时保存 config
    支持 PyTorch 模型和 sklearn/SVM 模型
    """
    def __init__(self, save_dir="model_project/result"):
        self.save_dir = save_dir
        os.makedirs(self.save_dir, exist_ok=True)
        self.best_val_loss = float('inf')
        self.best_model_path = None
        self.best_config_path = None

    def _generate_model_name(self, config_module):
        features = []
        if getattr(config_module, "USE_IMU", False): features.append("IMU")
        if getattr(config_module, "USE_DH", False): features.append("DH")
        if getattr(config_module, "USE_FK", False): features.append("FK")
        if getattr(config_module, "USE_JOINT", False): features.append("JOINT")
        if getattr(config_module, "USE_OTHER", False): features.append("OTHER")
        feature_str = "_".join(features) if features else "NOFEATURE"

        model_type = getattr(config_module, "MODEL_TYPE", "model")
        return f"{model_type}_{feature_str}"

    def save_best(self, model, config=None, epoch=None, val_loss=None):
        if val_loss is None:
            raise ValueError("val_loss 必须提供用于判断最优模型")

        if val_loss >= self.best_val_loss:
            return None

        if self.best_model_path and os.path.exists(self.best_model_path):
            os.remove(self.best_model_path)
        if self.best_config_path and os.path.exists(self.best_config_path):
            os.remove(self.best_config_path)

        self.best_val_loss = val_loss

        model_name = self._generate_model_name(config) if config else "model"
        if epoch is not None:
            model_name += f"_epoch{epoch}"

        model_path = os.path.join(self.save_dir, model_name)

        # 保存模型
        if hasattr(model, "state_dict"):
            model_path += ".pth"
            torch.save(model.state_dict(), model_path)
        else:
            model_path += ".joblib"
            joblib.dump(model, model_path)

        self.best_model_path = model_path
        print(f"Best model saved: {model_path}")

        # ===== 保持原 config 格式，仅追加字段 =====
        if config:
            config_dict = (
                config if isinstance(config, dict)
                else {k: getattr(config, k) for k in dir(config) if k.isupper()}
            )
        else:
            config_dict = {}

        # 追加“列”
        config_dict.update({
            "MODEL_PATH": model_path,
            "VAL_LOSS": val_loss,
            "EPOCH": epoch,
            "SAVE_TIME": datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        })

        json_path = os.path.join(self.save_dir, model_name + "_config.json")
        with open(json_path, "w") as f:
            json.dump(config_dict, f, indent=4)

        self.best_config_path = json_path
        print(f"Config updated: {json_path}")

        return model_path

