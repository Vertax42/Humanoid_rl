import re

def parse_imu_accel(line):
    """
    从日志行中提取 IMU 加速度数据 (x, y, z)。
    """
    # 使用正则表达式匹配 IMU 加速度数据
    match = re.search(r"Received imu_accel: \(([-\d.]+), ([-\d.]+), ([-\d.]+)\)", line)
    if match:
        x = float(match.group(1))
        y = float(match.group(2))
        z = float(match.group(3))
        return x, y, z
    return None

def calculate_bias(log_file):
    """
    计算 IMU 加速度数据的均值偏移量。
    """
    x_values = []
    y_values = []
    z_values = []

    # 读取日志文件
    with open(log_file, "r") as file:
        for line in file:
            accel_data = parse_imu_accel(line)
            if accel_data:
                x, y, z = accel_data
                x_values.append(x)
                y_values.append(y)
                z_values.append(z)

    # 计算均值
    if x_values and y_values and z_values:
        x_bias = sum(x_values) / len(x_values)
        y_bias = sum(y_values) / len(y_values)
        z_bias = sum(z_values) / len(z_values)
        return x_bias, y_bias, z_bias
    else:
        return None

if __name__ == "__main__":
    log_file = "humanoid_rl_node_Main_202501110023_037853_000.log"  # 替换为你的日志文件路径
    bias = calculate_bias(log_file)

    if bias:
        x_bias, y_bias, z_bias = bias
        print(f"IMU Bias Compensation:")
        print(f"  X: {x_bias:.6f}")
        print(f"  Y: {y_bias:.6f}")
        print(f"  Z: {z_bias:.6f}")
    else:
        print("No IMU acceleration data found in the log file.")
