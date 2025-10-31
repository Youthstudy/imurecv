import pandas as pd
from functools import reduce
import matplotlib.pyplot as plt

# ===================== ⚙️ 参数区 =====================
# 要同步的文件名
FILES = ["./imu_6C51C1_20251029_180108.csv",
        "./imu_8627ED_20251029_180108.csv",
        "./imu_8627F0_20251029_180108.csv",]

# 容差（毫秒）
TOLERANCE_MS = 50

# 输出文件名
OUTPUT_FILE = "merged.csv"
# =====================================================


def load_and_normalize(path):
    """读取 CSV，并自动识别时间格式"""
    df = pd.read_csv(path)

    # 自动找出包含 "time" 的列
    time_col = [c for c in df.columns if "time" in c.lower()]
    if not time_col:
        raise ValueError(f"{path} 中未找到时间列")
    time_col = time_col[0]

    # 转换为 datetime 格式
    df[time_col] = pd.to_datetime(df[time_col], errors="coerce")

    # 删除无效时间
    df = df.dropna(subset=[time_col])

    # 改名为统一列名
    df = df.rename(columns={time_col: "System_Time"})

    # 排序（merge_asof 要求有序）
    df = df.sort_values("System_Time").reset_index(drop=True)

    print(f"📁 {path}: {len(df)} 行, 时间范围 {df['System_Time'].min()} → {df['System_Time'].max()}")
    return df


def visualize_time_difference(dfs):
    """可视化文件时间差"""
    plt.figure(figsize=(10, 5))
    base_time = dfs[0]["System_Time"]
    for i, df in enumerate(dfs[1:], start=2):
        diff = (df["System_Time"].iloc[:len(base_time)] - base_time.iloc[:len(df)]).dt.total_seconds() * 1000
        plt.plot(diff, label=f"File{i} - File1")
    plt.axhline(0, color="gray", linestyle="--")
    plt.xlabel("Sample Index")
    plt.ylabel("Time Difference (ms)")
    plt.title("Time difference between files")
    plt.legend()
    plt.tight_layout()
    plt.show()


def sync_files(files, tolerance_ms):
    """核心同步逻辑"""
    dfs = [load_and_normalize(f) for f in files]

    print("\n🕐 正在执行近似对齐 (±{} ms)...".format(tolerance_ms))
    merged = reduce(
        lambda left, right: pd.merge_asof(
            left, right,
            on="System_Time",
            direction="nearest",
            tolerance=pd.Timedelta(milliseconds=tolerance_ms)
        ),
        dfs
    )

    merged.to_csv(OUTPUT_FILE, index=False)
    print(f"\n✅ 已生成同步文件: {OUTPUT_FILE}")
    print(f"📊 同步结果: {len(merged)} 行数据（容差 ±{tolerance_ms} ms）")
    return dfs, merged


if __name__ == "__main__":
    dfs, merged = sync_files(FILES, TOLERANCE_MS)
    if len(merged) == 0:
        print("\n⚠️ 同步结果为空，请尝试增大 TOLERANCE_MS（如 100~500 ms）或检查时间单位是否一致。")
    else:
        visualize_time_difference(dfs)



