import pandas as pd
import os
import re
from glob import glob
from functools import reduce

# ===================== ⚙️ 参数设置 =====================
# 数据所在路径
DATA_DIR = "./"  # 改为你的文件夹路径
# 容差（ms）
TOLERANCE_MS = 50
# 输出文件
OUTPUT_FILE = "merged_all.csv"
# =====================================================


def find_groups(data_dir):
    """根据时间戳模式分组，例如 20251106_183256"""
    pattern = re.compile(r"(\d{8}_\d{6})")
    files = glob(os.path.join(data_dir, "*.csv"))
    groups = {}
    for f in files:
        m = pattern.search(os.path.basename(f))
        if m:
            key = m.group(1)
            groups.setdefault(key, []).append(f)
    return groups


def load_imu_csv(path):
    """读取 IMU 文件并标准化时间列"""
    df = pd.read_csv(path)
    # 识别时间列
    time_col = None
    for c in df.columns:
        if "time" in c.lower():
            time_col = c
            break
    if not time_col:
        raise ValueError(f"{path} 中未找到时间列")

    df["System_Time"] = pd.to_datetime(df[time_col], errors="coerce")
    df = df.dropna(subset=["System_Time"])
    df = df.sort_values("System_Time").reset_index(drop=True)

    # 添加来源标记（imu编号）
    name = os.path.basename(path).split("_")[1]
    df.columns = [f"{col}_{name}" if col not in ["System_Time"] else col for col in df.columns]
    return df


def load_joint_csv(path):
    """读取关节文件并标准化时间列"""
    df = pd.read_csv(
        path,
        header=None,
        names=[
            "timestamp",
            "joint_id",
            "ret0", "ret1", "ret2",
            "p_des", "v_des", "kp", "kd", "t_ff"
        ],
    )
    df["System_Time"] = pd.to_datetime(df["timestamp"], errors="coerce")
    df = df.dropna(subset=["System_Time"])
    df = df.sort_values("System_Time").reset_index(drop=True)
    return df


def merge_group(files, tolerance_ms):
    """将同一时间组的 imu 与 joint 文件同步合并"""
    imu_files = [f for f in files if "imu" in os.path.basename(f).lower()]
    joint_files = [f for f in files if "joint" in os.path.basename(f).lower()]

    if not imu_files or not joint_files:
        print(f"⚠️ {files} 中缺少 imu 或 joint 文件，跳过。")
        return None

    # 加载并合并同组的 IMU 文件
    imu_dfs = [load_imu_csv(f) for f in imu_files]
    imu_merged = reduce(
        lambda left, right: pd.merge_asof(
            left, right,
            on="System_Time",
            direction="nearest",
            tolerance=pd.Timedelta(milliseconds=tolerance_ms),
        ),
        imu_dfs
    )

    # 加载并同步关节数据
    joint_df = pd.concat([load_joint_csv(f) for f in joint_files], ignore_index=True)
    joint_df = joint_df.sort_values("System_Time").reset_index(drop=True)

    # 与 IMU 合并
    merged = pd.merge_asof(
        joint_df, imu_merged,
        on="System_Time",
        direction="nearest",
        tolerance=pd.Timedelta(milliseconds=tolerance_ms)
    )

    return merged


def batch_merge(data_dir, tolerance_ms):
    """批量处理所有时间组"""
    groups = find_groups(data_dir)
    if not groups:
        print("⚠️ 未找到匹配的文件组。")
        return

    all_results = []
    print(f"🕐 共检测到 {len(groups)} 组数据，将逐组合并...\n")

    for key, files in sorted(groups.items()):
        print(f"🔹 合并时间组 {key} ...")
        merged = merge_group(files, tolerance_ms)
        if merged is not None and not merged.empty:
            merged["group"] = key
            all_results.append(merged)
            print(f"✅ {key} 合并完成，{len(merged)} 行。")
        else:
            print(f"⚠️ {key} 合并失败或为空。")

    if all_results:
        final_df = pd.concat(all_results, ignore_index=True)
        final_df.to_csv(OUTPUT_FILE, index=False, encoding="utf-8-sig")
        print(f"\n🎯 全部完成！输出文件: {OUTPUT_FILE}")
        print(f"📊 总行数: {len(final_df)}")
    else:
        print("❌ 未生成任何有效数据。")


# ========== 主程序入口 ==========
if __name__ == "__main__":
    batch_merge(DATA_DIR, TOLERANCE_MS)
