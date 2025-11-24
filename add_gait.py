import pandas as pd
import numpy as np
import sys

# ==============================
# 配置参数（请按需修改）
# ==============================
input_file = 'all_features.csv'
output_file = 'all_features_with_gait_phase.csv'

# 设置步态周期范围（按数据行索引，从 0 开始）
start_row = 527    # 步态周期开始的行索引（包含）
end_row = 527+570    # 步态周期结束的行索引（包含）

# ==============================
# 第一步：修复并读取 CSV（自动合并多行 header）
# ==============================
def read_csv_with_multipart_header(filepath):
    with open(filepath, 'r', encoding='utf-8') as f:
        lines = f.readlines()

    header_lines = []
    data_lines = []
    in_header = True

    for line in lines:
        stripped = line.strip()
        if not stripped:
            continue

        # 判断是否为数据行：尝试解析第一个字段为 float
        first_field = stripped.split(',')[0]
        try:
            float(first_field)
            in_header = False
        except ValueError:
            pass  # 仍可能是 header

        if in_header:
            header_lines.append(stripped.lstrip(','))
        else:
            data_lines.append(stripped)

    # 合并 header
    full_header = ''.join(header_lines)
    columns = [col for col in full_header.split(',') if col]

    # 用正确列名读取数据
    from io import StringIO
    data_text = '\n'.join(data_lines)
    df = pd.read_csv(StringIO(data_text), names=columns, header=None)
    return df

# ==============================
# 主程序
# ==============================
try:
    df = read_csv_with_multipart_header(input_file)
    print(f"✅ 成功加载数据，形状: {df.shape}")
except Exception as e:
    print(f"❌ 读取文件失败: {e}")
    sys.exit(1)

# 获取总行数
n_rows = len(df)

# 检查行号合法性
if start_row < 0 or end_row >= n_rows or start_row > end_row:
    raise ValueError(f"无效的行范围: start_row={start_row}, end_row={end_row}, 总行数={n_rows}")

# 初始化 gait_phase 列为 0
gait_phase = np.zeros(n_rows)

# 起始行之前：保持 0（默认已是 0）

# 起始行到结束行：线性插值 0 → 1
num_steps = end_row - start_row + 1
gait_phase[start_row : end_row + 1] = np.linspace(0, 1, num_steps)

# 结束行之后：设为 1
if end_row + 1 < n_rows:
    gait_phase[end_row + 1 :] = 1.0

# 添加到 DataFrame
df['gait_phase'] = gait_phase

# 保存结果
df.to_csv(output_file, index=False)
print(f"✅ 已成功添加 'gait_phase' 列并保存到 '{output_file}'")
print(f"   - 行 0 ~ {start_row-1}: gait_phase = 0")
print(f"   - 行 {start_row} ~ {end_row}: 线性插值 0 → 1")
print(f"   - 行 {end_row+1} ~ end: gait_phase = 1")

# 可选：打印前/后几行验证
print("\n前5行 gait_phase:")
print(df['gait_phase'].head())
print("后5行 gait_phase:")
print(df['gait_phase'].tail())