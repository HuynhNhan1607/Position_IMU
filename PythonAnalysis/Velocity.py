import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.widgets import SpanSelector
import numpy as np

# Đọc file CSV
df = pd.read_csv('logs/position_data_20250507_154125.csv')

# Tính tần số thực tế từ timestamp để hiển thị thông tin
timestamps = df['Timestamp_ms'].unique()  # Lấy các giá trị timestamp duy nhất
time_diffs = np.diff(timestamps)  # Tính khoảng cách giữa các timestamp liên tiếp

# Tính tần số lấy mẫu thực tế
avg_interval = np.mean(time_diffs)  # Khoảng thời gian trung bình (ms)
frequency = 1000 / avg_interval  # Chuyển thành Hz (1000ms = 1s)

# Tính tần số tổng thể thực tế
total_time = timestamps[-1] - timestamps[0]  # Tổng thời gian (ms)
total_samples = len(df)  # Tổng số mẫu
overall_frequency = total_samples / (total_time / 1000)  # Mẫu/giây

print(f"Khoảng thời gian trung bình thực tế giữa các mẫu: {avg_interval:.2f} ms")
print(f"Tần số lấy mẫu thực tế: {frequency:.2f} Hz")
print(f"Tổng thời gian: {total_time/1000:.2f} giây")
print(f"Tổng số mẫu: {total_samples}")
print(f"Tần số trung bình tổng thể: {overall_frequency:.2f} Hz")

# Tạo trục thời gian mới với khoảng cách cố định 40ms
sample_interval = 40.0  # ms - giả định tần số chính xác 25Hz
time_axis = np.arange(0, len(df)) * sample_interval

# Tạo figure với 2 đồ thị xếp dọc
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8), sharex=True)
plt.subplots_adjust(hspace=0.3)
fig.suptitle(f'Phân tích dữ liệu vị trí và vận tốc (25Hz)', fontsize=16)

# Hiển thị thông tin tần số trong một text box
freq_info = (f"Tần số lấy mẫu thực tế: {frequency:.2f} Hz\n"
             f"Tần số tổng thể thực tế: {overall_frequency:.2f} Hz\n"
             f"Biểu đồ: 25Hz (40ms/mẫu)")
fig.text(0.02, 0.95, freq_info, fontsize=10, 
         bbox=dict(boxstyle="round,pad=0.5", facecolor='white', alpha=0.7))

# 1. Vị trí theo thời gian (sử dụng trục thời gian mới 40ms/mẫu)
ax1.plot(time_axis, df['PositionX'], 'r-', label='X')
ax1.plot(time_axis, df['PositionY'], 'g-', label='Y')
ax1.set_title('Vị trí theo thời gian')
ax1.set_ylabel('Vị trí (m)')
ax1.legend()
ax1.grid(True)

# 2. Vận tốc theo thời gian (sử dụng trục thời gian mới 40ms/mẫu)
ax2.plot(time_axis, df['VelocityX'], 'r-', label='X')
ax2.plot(time_axis, df['VelocityY'], 'g-', label='Y')
ax2.set_title('Vận tốc theo thời gian')
ax2.set_xlabel('Thời gian (ms)')
ax2.set_ylabel('Vận tốc (m/s)')
ax2.legend()
ax2.grid(True)

# Đặt giới hạn ban đầu
xmin, xmax = time_axis[0], time_axis[-1]
ax1.set_xlim(xmin, xmax)
ax2.set_xlim(xmin, xmax)

# Hàm xử lý zoom đồng thời
def onselect(xmin, xmax):
    ax1.set_xlim(xmin, xmax)
    ax2.set_xlim(xmin, xmax)
    fig.canvas.draw_idle()

# Thêm SpanSelector cho zoom ngang
span = SpanSelector(
    ax2,
    onselect,
    'horizontal',
    useblit=True,
    props=dict(alpha=0.5, facecolor='lightblue'),
    interactive=True,
    drag_from_anywhere=True
)

# Hiển thị hướng dẫn sử dụng
fig.text(0.5, 0.01, 'Kéo chuột ngang để zoom, double-click để reset', 
         ha='center', va='bottom', fontsize=10, 
         bbox=dict(boxstyle="round,pad=0.5", facecolor='lightgrey', alpha=0.5))

# Hàm xử lý double-click để reset zoom
def on_double_click(event):
    if event.dblclick:
        ax1.set_xlim(xmin, xmax)
        ax2.set_xlim(xmin, xmax)
        fig.canvas.draw_idle()

fig.canvas.mpl_connect('button_press_event', on_double_click)

plt.tight_layout(rect=[0, 0.03, 1, 0.96])
plt.show()