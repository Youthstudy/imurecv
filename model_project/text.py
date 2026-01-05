import torch

checkpoint = torch.load("model_project/result/lstm_IMU_JOINT_epoch4.pth", map_location="cpu")
print("类型:", type(checkpoint))
print("\n权重层:")
for key, value in checkpoint.items():
    print(f"  {key}: {value.shape}")