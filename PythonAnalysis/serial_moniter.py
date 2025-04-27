# serial_monitor.py
import serial
import json
import time
import re
from collections import deque
import os
import csv
from datetime import datetime

# Đặt backend tương tác
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

class BNO055DataProcessor:
    def __init__(self, max_points=150):
        # Thời gian bắt đầu để tính timestamp tương đối
        self.start_time = time.time()
        
        # Định nghĩa kích thước buffer
        self.MAX_POINTS = max_points
        
        # Buffer dữ liệu
        self.accel_x_data = deque(maxlen=self.MAX_POINTS)
        self.accel_y_data = deque(maxlen=self.MAX_POINTS)
        self.accel_z_data = deque(maxlen=self.MAX_POINTS)
        self.heading_data = deque(maxlen=self.MAX_POINTS)
        self.pitch_data = deque(maxlen=self.MAX_POINTS)
        self.roll_data = deque(maxlen=self.MAX_POINTS)
        self.gyro_x_data = deque(maxlen=self.MAX_POINTS)
        self.gyro_y_data = deque(maxlen=self.MAX_POINTS)
        self.gyro_z_data = deque(maxlen=self.MAX_POINTS)
        self.status_data = deque(maxlen=self.MAX_POINTS)  

        self.position_data = deque(maxlen=self.MAX_POINTS)
        self.velocity_data = deque(maxlen=self.MAX_POINTS)

        # Buffer nhỏ hơn cho tính tần số chính xác
        self.packet_timestamps = deque(maxlen=25)
        self.packet_frequency = 0.0
        
        # Giá trị hiện tại
        self.current_values = {
            "accel": [0.0, 0.0, 0.0],
            "euler": [0.0, 0.0, 0.0],
            "gyro": [0.0, 0.0, 0.0],
            "status": "stationary"  
        }
        
        # CSV logging
        self.csv_file = None
        self.csv_writer = None
        
        # CSV logging riêng cho vị trí
        self.position_csv_file = None
        self.position_csv_writer = None

        # Khởi tạo với giá trị 0
        for i in range(5):
            self.accel_x_data.append(0)
            self.accel_y_data.append(0)
            self.accel_z_data.append(0)
            self.heading_data.append(0)
            self.pitch_data.append(0)
            self.roll_data.append(0)
            self.gyro_x_data.append(0)
            self.gyro_y_data.append(0)
            self.gyro_z_data.append(0)
            self.position_data.append([0.0, 0.0])
            self.velocity_data.append([0.0, 0.0])
            self.status_data.append("stationary")
        
        # Đối tượng serial
        self.ser = None
        self.connected = False
    
    def connect_serial(self, port, baudrate):
        """Kết nối tới cổng serial với cấu hình tối ưu để không mất gói tin"""
        try:
            # Tăng kích thước buffer nhận để đảm bảo không bị tràn buffer
            # Giảm timeout để đọc dữ liệu thường xuyên hơn
            self.ser = serial.Serial(
                port, 
                baudrate, 
                timeout=0.01,  # Giảm timeout để đọc thường xuyên hơn
                write_timeout=1,
                inter_byte_timeout=None,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                xonxoff=False,
                rtscts=False,
                dsrdtr=False
            )
            # Tăng kích thước buffer để tránh mất dữ liệu
            if hasattr(self.ser, 'set_buffer_size'):
                self.ser.set_buffer_size(rx_size=131072, tx_size=131072)  # 128KB buffer
                
            self.connected = True
            print(f"Đã kết nối với {port} ở {baudrate} baud với buffer lớn")
            return True
        except Exception as e:
            print(f"Lỗi kết nối: {e}")
            self.connected = False
            return False
    
    def disconnect(self):
        """Đóng kết nối serial"""
        if self.ser and self.ser.is_open:
            self.ser.close()
            self.connected = False
            print("Đã ngắt kết nối")
    
    def get_relative_timestamp_ms(self):
        """Lấy timestamp tương đối tính từ lúc bắt đầu (ms)"""
        return int((time.time() - self.start_time) * 1000)
    
    def calculate_packet_frequency(self):
        """Tính tần số đơn giản với cửa sổ cố định 20 gói tin"""
        WINDOW_SIZE = 20  # Cố định 20 gói tin
        
        # Đảm bảo có đủ dữ liệu
        if len(self.packet_timestamps) < WINDOW_SIZE:
            return
        
        # Lấy thời gian gói đầu và gói thứ 20
        time_first = self.packet_timestamps[0]
        time_last = self.packet_timestamps[-1]
        
        # Tính khoảng thời gian
        time_diff = time_last - time_first
        
        if time_diff > 0:
            # Tính tần số = (số gói - 1) / thời gian
            # (số gói - 1) vì có n gói thì có (n-1) khoảng
            self.packet_frequency = (WINDOW_SIZE - 1) / time_diff
    
    def read_and_update_data(self):
        """Đọc và cập nhật dữ liệu từ serial với xử lý chính xác newline"""
        if self.ser is None or not self.ser.is_open:
            return False
        
        data_updated = False
        
        if not hasattr(self, 'buffer_remainder'):
            self.buffer_remainder = ''

        try:
            if self.ser.in_waiting > 0:
                # Đọc dữ liệu mới
                raw_data = self.ser.read(self.ser.in_waiting)
                
                # Chuyển đổi dữ liệu thành chuỗi
                text_data = raw_data.decode('utf-8', errors='ignore')
                
                # Nối với dữ liệu chưa hoàn chỉnh từ lần đọc trước
                text_data = self.buffer_remainder + text_data
                
                # Tách dữ liệu thành từng dòng vì mỗi JSON kết thúc bằng \n
                lines = text_data.split('\n')
                
                # Phần còn lại có thể là một dòng không hoàn chỉnh
                self.buffer_remainder = lines[-1]
                
                # Xử lý các dòng hoàn chỉnh
                for line in lines[:-1]:
                    if not line.strip():
                        continue
                    
                    # Tìm JSON trong dòng
                    match = re.search(r'({.*})', line)
                    if not match:
                        continue
                    
                    json_str = match.group(1)
                    try:
                        data = json.loads(json_str)
                        
                        if 'type' in data:
                            message_type = data['type']
                            
                            if message_type == "bno055":
                                # Xử lý dữ liệu bno055
                                if 'data' in data:
                                    data_content = data['data']
                                    
                                    if 'lin_accel' in data_content:
                                        accel = data_content['lin_accel']
                                        self.current_values["accel"] = accel
                                        self.accel_x_data.append(accel[0])
                                        self.accel_y_data.append(accel[1])
                                        self.accel_z_data.append(accel[2])
                                    
                                    if 'euler' in data_content:
                                        euler = data_content['euler']
                                        self.current_values["euler"] = euler
                                        self.heading_data.append(euler[0])
                                        self.pitch_data.append(euler[1])
                                        self.roll_data.append(euler[2])
                                    
                                    if 'gyro_raw' in data_content:
                                        gyro = data_content['gyro_raw']
                                        self.current_values["gyro"] = gyro
                                        self.gyro_x_data.append(gyro[0])
                                        self.gyro_y_data.append(gyro[1])
                                        self.gyro_z_data.append(gyro[2])

                                    if 'status' in data_content:
                                        motion = data_content['status']
                                        self.current_values["status"] = motion
                                        self.status_data.append(motion)
                                    
                                    data_updated = True
                            
                            elif message_type == "state":
                                # Xử lý dữ liệu status
                                if 'data' in data:
                                    status_data = data['data']
                                    
                                    if 'position' in status_data:
                                        position = status_data['position']
                                        self.current_values["position"] = position
                                        self.position_data.append(position)
                                    
                                    if 'velocity' in status_data:
                                        velocity = status_data['velocity']
                                        self.current_values["velocity"] = velocity
                                        self.velocity_data.append(velocity)
                                    
                                    # Ghi dữ liệu vị trí vào file CSV riêng
                                    if self.position_csv_writer is not None:
                                        self.log_position_to_csv()
                                    
                                    data_updated = True
                        
                        # Ghi dữ liệu vào CSV
                        if self.csv_writer is not None:
                            self.log_data_to_csv()
                    except json.JSONDecodeError:
                        print(f"Lỗi phân tích JSON: {json_str}")
        
        except Exception as e:
            print(f"Lỗi khi đọc dữ liệu: {e}")
        
        return data_updated
    
    def get_all_data(self):
        """Lấy tất cả dữ liệu cho hiển thị"""
        return {
            "connected": self.connected,
            "current_values": self.current_values,
            "accel_x_data": list(self.accel_x_data),
            "accel_y_data": list(self.accel_y_data),
            "accel_z_data": list(self.accel_z_data),
            "heading_data": list(self.heading_data),
            "pitch_data": list(self.pitch_data),
            "roll_data": list(self.roll_data),
            "gyro_x_data": list(self.gyro_x_data),
            "gyro_y_data": list(self.gyro_y_data),
            "gyro_z_data": list(self.gyro_z_data),
            "status_data": list(self.status_data), 
            "packet_frequency": self.packet_frequency
        }
    
    def start_csv_logging(self):
        """Bắt đầu ghi log dữ liệu vào file CSV"""
        try:
            # Tạo thư mục logs nếu chưa tồn tại
            os.makedirs('logs', exist_ok=True)
            
            # Tạo tên file
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"logs/bno055_data_{timestamp}.csv"
            
            # Mở file và tạo writer
            self.csv_file = open(filename, 'w', newline='')
            self.csv_writer = csv.writer(self.csv_file)
            
            # Ghi header
            self.csv_writer.writerow([
                'Timestamp_ms', 
                'AccelX', 'AccelY', 'AccelZ', 
                'Heading', 'Pitch', 'Roll',
                'GyroX', 'GyroY', 'GyroZ',
                'Status'
            ])
            
            print(f"Đã bắt đầu ghi log tại: {os.path.abspath(filename)}")
            return True
        except Exception as e:
            print(f"Lỗi khi bắt đầu CSV logging: {e}")
            return False
    
    def start_position_logging(self):
        """Bắt đầu ghi log dữ liệu vị trí vào file CSV riêng"""
        try:
            # Tạo thư mục logs nếu chưa tồn tại
            os.makedirs('logs', exist_ok=True)
            
            # Tạo tên file
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"logs/position_data_{timestamp}.csv"
            
            # Mở file và tạo writer
            self.position_csv_file = open(filename, 'w', newline='')
            self.position_csv_writer = csv.writer(self.position_csv_file)
            
            # Ghi header
            self.position_csv_writer.writerow([
                'Timestamp_ms', 
                'PositionX', 'PositionY',
                'VelocityX', 'VelocityY',
                'Status'
            ])
            
            print(f"Đã bắt đầu ghi log vị trí tại: {os.path.abspath(filename)}")
            return True
        except Exception as e:
            print(f"Lỗi khi bắt đầu Position CSV logging: {e}")
            return False
    
    def log_data_to_csv(self):
        """Ghi dữ liệu hiện tại vào file CSV"""
        if self.csv_writer is None:
            return
        
        try:
            # Lấy timestamp tương đối (ms)
            rel_timestamp = self.get_relative_timestamp_ms()
            
            # Ghi một dòng dữ liệu
            self.csv_writer.writerow([
                rel_timestamp,
                self.current_values["accel"][0],
                self.current_values["accel"][1],
                self.current_values["accel"][2],
                self.current_values["euler"][0],
                self.current_values["euler"][1],
                self.current_values["euler"][2],
                self.current_values["gyro"][0],
                self.current_values["gyro"][1],
                self.current_values["gyro"][2],
                self.current_values["status"]
            ])
            
            # Flush để đảm bảo dữ liệu được ghi ngay lập tức
            self.csv_file.flush()
        except Exception as e:
            print(f"Lỗi ghi CSV: {e}")
    
    def log_position_to_csv(self):
        """Ghi dữ liệu vị trí và vận tốc vào file CSV riêng"""
        if self.position_csv_writer is None:
            return
        
        try:
            # Lấy timestamp tương đối (ms)
            rel_timestamp = self.get_relative_timestamp_ms()
            
            # Lấy giá trị vị trí và vận tốc
            position = self.current_values.get("position", [0.0, 0.0])
            velocity = self.current_values.get("velocity", [0.0, 0.0])
            status = self.current_values.get("status", "stationary")
            
            # Ghi một dòng dữ liệu
            self.position_csv_writer.writerow([
                rel_timestamp,
                position[0],
                position[1],
                velocity[0],
                velocity[1],
                status
            ])
            
            # Flush để đảm bảo dữ liệu được ghi ngay lập tức
            self.position_csv_file.flush()
        except Exception as e:
            print(f"Lỗi ghi Position CSV: {e}")
    
    def stop_csv_logging(self):
        """Dừng ghi log và đóng file CSV"""
        if self.csv_file:
            self.csv_file.close()
            self.csv_file = None
            self.csv_writer = None
            print("Đã dừng ghi CSV")
    
    def stop_position_logging(self):
        """Dừng ghi log vị trí và đóng file CSV"""
        if self.position_csv_file:
            self.position_csv_file.close()
            self.position_csv_file = None
            self.position_csv_writer = None
            print("Đã dừng ghi log vị trí")

def main():
    # Giá trị mặc định
    port = 'COM4'
    baud = 115200
    
    # Khởi tạo processor
    data_processor = BNO055DataProcessor()
    
    # Kết nối serial
    data_processor.connect_serial(port, baud)
    
    # Luôn bật CSV logging
    data_processor.start_csv_logging()
    data_processor.start_position_logging()
    
    # Cửa sổ đồ thị
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8))
    fig.suptitle("BNO055 Sensor Data")
    
    # Đồ thị gia tốc
    ax1.set_title("Linear Acceleration")
    ax1.set_ylabel("Acceleration (m/s²)")
    ax1.set_ylim(-10, 10)
    accel_x_line, = ax1.plot([], [], 'r-', label='X-axis')
    accel_y_line, = ax1.plot([], [], 'g-', label='Y-axis')
    accel_z_line, = ax1.plot([], [], 'b-', label='Z-axis')
    ax1.legend()
    ax1.grid(True)
    
    # Đồ thị hướng
    ax2.set_title("Orientation")
    ax2.set_ylabel("Angle (degrees)")
    ax2.set_ylim(-361, 361)
    heading_line, = ax2.plot([], [], 'orange', label='Heading')
    pitch_line, = ax2.plot([], [], 'purple', label='Pitch')
    roll_line, = ax2.plot([], [], 'brown', label='Roll')
    ax2.legend()
    ax2.grid(True)
    
    # Text trạng thái
    status_text = fig.text(0.02, 0.01, "", fontsize=10)
    
    motion_text = fig.text(0.85, 0.07, "STATIONARY", fontsize=16, 
                    bbox=dict(facecolor='red', alpha=0.5))
    
    position_text = fig.text(0.5, 0.01, "", fontsize=10, ha='left')

    def update_animation(frame):
        """Cập nhật đồ thị"""
        # Đọc dữ liệu mới
        data_processor.read_and_update_data()
        
        # Lấy tất cả dữ liệu cho hiển thị
        data = data_processor.get_all_data()
        
        # Cập nhật đồ thị
        accel_x_line.set_data(range(len(data["accel_x_data"])), data["accel_x_data"])
        accel_y_line.set_data(range(len(data["accel_y_data"])), data["accel_y_data"])
        accel_z_line.set_data(range(len(data["accel_z_data"])), data["accel_z_data"])
        
        heading_line.set_data(range(len(data["heading_data"])), data["heading_data"])
        pitch_line.set_data(range(len(data["pitch_data"])), data["pitch_data"])
        roll_line.set_data(range(len(data["roll_data"])), data["roll_data"])
        
        # Điều chỉnh trục x
        ax1.set_xlim(0, len(data["accel_x_data"]))
        ax2.set_xlim(0, len(data["heading_data"]))
        
        # Cập nhật text trạng thái (bên trái)
        status = "CONNECTED" if data["connected"] else "DISCONNECTED"
        accel = data["current_values"]["accel"]
        euler = data["current_values"]["euler"]
        gyro = data["current_values"]["gyro"]
        
        status_text.set_text(
            f"Status: {status}\n"
            f"Data Rate: {data['packet_frequency']:.1f} Hz\n"
            f"Acceleration (m/s²): X={accel[0]:.3f}, Y={accel[1]:.3f}, Z={accel[2]:.3f}\n"
            f"Orientation (deg): Heading={euler[0]:.1f}, Pitch={euler[1]:.1f}, Roll={euler[2]:.1f}\n"
            f"Gyroscope (raw): X={gyro[0]:.3f}, Y={gyro[1]:.3f}, Z={gyro[2]:.3f}"
        )
        
        # Cập nhật text position (bên phải)
        position = data["current_values"].get("position", [0.0, 0.0])
        velocity = data["current_values"].get("velocity", [0.0, 0.0])
        
        position_text.set_text(
            f"Position (m):\n"
            f"  X = {position[0]:.4f}\n"
            f"  Y = {position[1]:.4f}\n"
            f"Velocity (m/s):\n"
            f"  X = {velocity[0]:.4f}\n"
            f"  Y = {velocity[1]:.4f}"
        )
        
        # Cập nhật trạng thái chuyển động
        motion_status = data["current_values"]["status"]
        if motion_status == "moving":
            motion_text.set_text("MOVING")
            motion_text.set_bbox(dict(facecolor='green', alpha=0.5))
        else:
            motion_text.set_text("STATIONARY")
            motion_text.set_bbox(dict(facecolor='red', alpha=0.5))
        
        # Nhớ trả về tất cả các đối tượng đồ thị được cập nhật
        return accel_x_line, accel_y_line, accel_z_line, heading_line, pitch_line, roll_line, status_text, motion_text, position_text
    
    # Layout
    plt.tight_layout()
    plt.subplots_adjust(bottom=0.15)
    
    # Animation
    ani = FuncAnimation(fig, update_animation, interval=200, cache_frame_data=False)
    
    # Hiển thị
    try:
        plt.show()
    except Exception as e:
        print(f"Lỗi hiển thị: {e}")
    
    # Dọn dẹp
    data_processor.stop_csv_logging()
    data_processor.stop_position_logging()
    data_processor.disconnect()

if __name__ == "__main__":
    main()