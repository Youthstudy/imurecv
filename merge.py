import pandas as pd
import os
import glob
import re
from functools import reduce

# ===================== ⚙️ 参数设置 =====================
base_path = r"F:\3.biye\1_code\imurecv\data"
output_dir = os.path.join(base_path, "merged")
os.makedirs(output_dir, exist_ok=True)
tolerance_ms = 25  # 时间容差（毫秒）
# =======================================================

def find_groups():
    """根据文件名中的时间戳分组"""
    files = glob.glob(os.path.join(base_path, "*.csv"))
    pattern = re.compile(r"(\d{8}_\d{6})")
    groups = {}
    for f in files:
        match = pattern.search(f)
        if match:
            key = match.group(1)
            groups.setdefault(key, []).append(f)
    return groups


def load_imu(file, suffix):
    """加载并标记 IMU 数据"""
    df = pd.read_csv(file)
    df["System_Time"] = pd.to_datetime(df["System_Time"], errors="coerce")
    df = df.dropna(subset=["System_Time"]).sort_values("System_Time")
    rename_dict = {col: f"{col}_{suffix}" for col in df.columns if col not in ["System_Time"]}
    df = df.rename(columns=rename_dict)
    return df


def load_joint(file):
    """加载并处理 Joint 文件"""
    df = pd.read_csv(file)
    df["timestamp"] = pd.to_datetime(df["timestamp"], errors="coerce")
    df = df.dropna(subset=["timestamp"])
    df = df.pivot_table(index="timestamp", columns="joint_id",
                        values=["ret0", "ret1", "ret2", "p_des", "v_des", "kp", "kd", "t_ff"])
    df.columns = [f"{a}_joint{int(b)}" for a, b in df.columns]
    df = df.reset_index().sort_values("timestamp")
    return df


def merge_group(group_files, group_name):
    """对同一时间组的 IMU + Joint 进行同步合并"""
    imu_files = [f for f in group_files if "imu_" in os.path.basename(f)]
    joint_files = [f for f in group_files if "joint_" in os.path.basename(f)]

    if len(imu_files) == 0 or len(joint_files) == 0:
        print(f"⚠️ 跳过 {group_name}（未找到完整IMU或Joint文件）")
        return

    # === 加载 IMU 数据 ===
    imu_dfs = []
    for i, f in enumerate(sorted(imu_files), start=1):
        imu_dfs.append(load_imu(f, i))
    imu_merged = reduce(
        lambda left, right: pd.merge_asof(
            left, right,
            on="System_Time",
            direction="nearest",
            tolerance=pd.Timedelta(milliseconds=tolerance_ms)
        ),
        imu_dfs
    )

    # === 加载 Joint 数据 ===
    joint_df = load_joint(joint_files[0])

    # === 同步 IMU 与 Joint ===
    merged = pd.merge_asof(
        imu_merged.sort_values("System_Time"),
        joint_df.sort_values("timestamp"),
        left_on="System_Time",
        right_on="timestamp",
        direction="nearest",
        tolerance=pd.Timedelta(milliseconds=tolerance_ms)
    )

    merged = merged.dropna(subset=["timestamp"])
    merged = merged.drop(columns=["timestamp"])
    output_file = os.path.join(output_dir, f"merged_{group_name}.csv")
    merged.to_csv(output_file, index=False, encoding="utf-8-sig")

    print(f"✅ 已输出: {output_file} ({len(merged)} 行)")


if __name__ == "__main__":
    groups = find_groups()
    print(f"🧩 发现 {len(groups)} 个时间组:")
    for g, fs in groups.items():
        print(f"  {g}: {len(fs)} 文件")
    print("=====================================\n")

    for group_name, group_files in groups.items():
        merge_group(group_files, group_name)

    print("\n🎉 所有时间组合并完成！")
