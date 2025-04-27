import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns

# Đặt kiểu font có hỗ trợ tiếng Việt
plt.rcParams['font.sans-serif'] = ['Arial Unicode MS', 'DejaVu Sans', 'sans-serif']

# Đọc file CSV
df = pd.read_csv('logs/bno055_data_20250424_170253.csv')

# Hiển thị thông tin cơ bản
print("Số mẫu trong dataset:", len(df))
print("Các cột trong dataset:", df.columns.tolist())

# Xác định các cột liên quan đến Euler, Acceleration và Gyroscope
# Gán thủ công các cột Euler
euler_cols = ['Heading', 'Pitch', 'Roll']

# Gán thủ công các cột accelerometer
accel_cols = ['AccelX', 'AccelY', 'AccelZ']

# Gán thủ công các cột gyroscope
gyro_cols = ['GyroX', 'GyroY', 'GyroZ']

# Xác định các mức kích thước mẫu cần đánh giá
max_samples = len(df)
sample_sizes = [10, 20, 50, 100, 200, 500, 1000, 2000, 5000]
sample_sizes = [size for size in sample_sizes if size <= max_samples]
if max_samples not in sample_sizes and max_samples > sample_sizes[-1]:
    sample_sizes.append(max_samples)

# Các hàm tính toán
def calculate_stats_by_sample_size(data, columns, sample_sizes):
    means = {col: [] for col in columns}
    stds = {col: [] for col in columns}
    
    for size in sample_sizes:
        sample_data = data.iloc[:size]
        for col in columns:
            means[col].append(sample_data[col].mean())
            stds[col].append(sample_data[col].std())
    
    return means, stds

# Tính toán thống kê cho từng nhóm cảm biến
euler_means, euler_stds = calculate_stats_by_sample_size(df, euler_cols, sample_sizes)
accel_means, accel_stds = calculate_stats_by_sample_size(df, accel_cols, sample_sizes)
gyro_means, gyro_stds = calculate_stats_by_sample_size(df, gyro_cols, sample_sizes)

# Vẽ đồ thị
def create_plots(means, stds, title, sample_sizes):
    fig, axes = plt.subplots(1, 2, figsize=(15, 6))
    
    # Đồ thị Mean
    for col in means.keys():
        axes[0].plot(sample_sizes, means[col], 'o-', label=col)
    axes[0].set_xlabel('Số lượng mẫu')
    axes[0].set_ylabel('Giá trị trung bình')
    axes[0].set_title(f'Giá trị trung bình của {title}')
    axes[0].grid(True)
    axes[0].legend()
    
    # Đồ thị Standard Deviation
    for col in stds.keys():
        axes[1].plot(sample_sizes, stds[col], 'o-', label=col)
    axes[1].set_xlabel('Số lượng mẫu')
    axes[1].set_ylabel('Độ lệch chuẩn')
    axes[1].set_title(f'Độ lệch chuẩn của {title}')
    axes[1].grid(True)
    axes[1].legend()
    
    plt.tight_layout()
    return fig

# Tạo đồ thị cho từng nhóm dữ liệu
if euler_cols:
    euler_fig = create_plots(euler_means, euler_stds, 'Euler', sample_sizes)
    
if accel_cols:
    accel_fig = create_plots(accel_means, accel_stds, 'Acceleration', sample_sizes)
    
if gyro_cols:
    gyro_fig = create_plots(gyro_means, gyro_stds, 'Gyroscope', sample_sizes)

# Tạo bảng thống kê để dễ dàng theo dõi
def create_stats_table(means, stds, sample_sizes):
    results = {}
    for col in means.keys():
        col_name = col.replace('_', ' ').title()
        results[f"{col_name} (Mean)"] = means[col]
        results[f"{col_name} (Std)"] = stds[col]
    
    return pd.DataFrame(results, index=sample_sizes)

# In bảng thống kê
print("\nBảng thống kê Euler:")
if euler_cols:
    euler_table = create_stats_table(euler_means, euler_stds, sample_sizes)
    print(euler_table)

print("\nBảng thống kê Acceleration:")
if accel_cols:
    accel_table = create_stats_table(accel_means, accel_stds, sample_sizes)
    print(accel_table)

print("\nBảng thống kê Gyroscope:")
if gyro_cols:
    gyro_table = create_stats_table(gyro_means, gyro_stds, sample_sizes)
    print(gyro_table)

plt.show()