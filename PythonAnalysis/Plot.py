import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.widgets import Slider

# Đọc dữ liệu từ file CSV
def load_data(file_path):
    df = pd.read_csv(file_path)
    
    # Tạo cột thời gian nếu chưa có
    if 'Time' not in df.columns:
        # Giả sử có cột thời gian dạng Timestamp hoặc tạo mới từ index
        if 'Timestamp' in df.columns:
            df['Time'] = pd.to_datetime(df['Timestamp'])
        else:
            df['Time'] = np.arange(len(df)) / 100.0  # Giả định tần số lấy mẫu 100Hz
    
    return df

# Tạo hình vẽ với 3 biểu đồ và chức năng zoom đồng bộ
def plot_sensor_data(df, euler_cols, accel_cols, gyro_cols):
    # Tạo figure và axes
    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(12, 10), sharex=True)
    plt.subplots_adjust(bottom=0.15)  # Để chỗ cho slider
    
    # Vẽ dữ liệu Euler angles
    lines_euler = []
    for col in euler_cols:
        if col in df.columns:
            line, = ax1.plot(df['Time'], df[col], label=col)
            lines_euler.append(line)
    ax1.set_title('Euler Angles (degrees)')
    ax1.set_ylabel('Angle')
    ax1.grid(True)
    ax1.legend(loc='upper right')
    
    # Vẽ dữ liệu Accelerometer
    lines_accel = []
    for col in accel_cols:
        if col in df.columns:
            line, = ax2.plot(df['Time'], df[col], label=col)
            lines_accel.append(line)
    ax2.set_title('Accelerometer (m/s²)')
    ax2.set_ylabel('Acceleration')
    ax2.grid(True)
    ax2.legend(loc='upper right')
    
    # Vẽ dữ liệu Gyroscope
    lines_gyro = []
    for col in gyro_cols:
        if col in df.columns:
            line, = ax3.plot(df['Time'], df[col], label=col)
            lines_gyro.append(line)
    ax3.set_title('Gyroscope (rad/s)')
    ax3.set_xlabel('Time')
    ax3.set_ylabel('Angular velocity')
    ax3.grid(True)
    ax3.legend(loc='upper right')
    
    # Thêm thanh trượt điều khiển zoom
    ax_slider = plt.axes([0.1, 0.05, 0.8, 0.03])
    time_range = df['Time'].max() - df['Time'].min()
    slider_window = time_range * 0.2  # Default zoom window là 20% của toàn bộ dữ liệu
    
    slider = Slider(
        ax_slider, 'Zoom Window', 
        df['Time'].min(), df['Time'].max() - slider_window,
        valinit=df['Time'].min()
    )
    
    # Hàm cập nhật khi thay đổi thanh trượt
    def update(val):
        start = slider.val
        end = start + slider_window
        for ax in [ax1, ax2, ax3]:
            ax.set_xlim(start, end)
        fig.canvas.draw_idle()
    
    slider.on_changed(update)
    
    # Thêm tính năng zoom đồng bộ dùng mouse wheel
    def on_scroll(event):
        if event.inaxes:
            axs = [ax1, ax2, ax3]
            current_xlim = event.inaxes.get_xlim()
            xdata = event.xdata
            
            base_scale = 1.5
            # Zoom in
            if event.button == 'up':
                scale_factor = 1 / base_scale
            # Zoom out
            elif event.button == 'down':
                scale_factor = base_scale
            else:
                scale_factor = 1
            
            new_width = (current_xlim[1] - current_xlim[0]) * scale_factor
            relx = (xdata - current_xlim[0]) / (current_xlim[1] - current_xlim[0])
            
            for ax in axs:
                ax.set_xlim([xdata - new_width * relx, xdata + new_width * (1-relx)])
            
            fig.canvas.draw_idle()
    
    fig.canvas.mpl_connect('scroll_event', on_scroll)
    
    # Vẽ status lên gyro plot (ax3)
    if 'Status' in df.columns:
        # Định nghĩa màu cụ thể cho từng trạng thái
        status_colors = {'moving': 'red', 'stationary': 'green'}
        
        # Chuyển đổi status thành số để vẽ màu nền
        status_map = {s: i for i, s in enumerate(df['Status'].unique())}
        status_numeric = df['Status'].map(status_map)
        prev_status = df['Status'].iloc[0]
        start_idx = 0
        
        for i, val in enumerate(df['Status']):
            if val != prev_status or i == len(df['Status']) - 1:
                end_idx = i if val != prev_status else i + 1
                ax3.axvspan(df['Time'].iloc[start_idx], df['Time'].iloc[end_idx-1], 
                            color=status_colors.get(prev_status, 'gray'), alpha=0.15)
                start_idx = i
                prev_status = val
                
        # Thêm chú thích cho status
        handles = [plt.Rectangle((0,0),1,1, color=status_colors.get(s, 'gray'), alpha=0.3) for s in status_map.keys()]
        ax3.legend(handles, status_map.keys(), loc='upper left', title='Status')

    # Hiển thị plot
    plt.tight_layout(rect=[0, 0.1, 1, 0.98])
    plt.show()

# Main function
def main():
    # Tên file dữ liệu
    file_path = "logs/bno055_data_20250507_154125.csv"
    
    # Đọc dữ liệu
    df = load_data(file_path)
    
    # Định nghĩa các cột dữ liệu
    euler_cols = ['Heading', 'Pitch', 'Roll']
    accel_cols = ['AccelX', 'AccelY', 'AccelZ']
    gyro_cols = ['GyroX', 'GyroY', 'GyroZ']
    
    # Vẽ đồ thị
    plot_sensor_data(df, euler_cols, accel_cols, gyro_cols)

if __name__ == "__main__":
    main()