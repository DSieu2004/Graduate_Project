# import tkinter as tk
# from tkinter import ttk, messagebox
# import serial
# import threading
# import time
# import collections
# import math # Thư viện tính toán góc

# import matplotlib
# matplotlib.use('TkAgg')
# import matplotlib.pyplot as plt
# import matplotlib.animation as animation
# from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

# # ================= GLOBAL VARIABLES ==================
# BAUD_RATE = 115200
# SERIAL_PORT = "/dev/serial0" # Đổi thành COMx nếu chạy trên Windows nhé
# MAX_POINTS = 100

# running = False
# ser = None

# time_data = collections.deque(maxlen=MAX_POINTS)
# tar_r_data = collections.deque(maxlen=MAX_POINTS)
# act_r_data = collections.deque(maxlen=MAX_POINTS)
# tar_l_data = collections.deque(maxlen=MAX_POINTS)
# act_l_data = collections.deque(maxlen=MAX_POINTS)

# start_time = 0
# actual_vr = 0.0
# actual_vl = 0.0

# # THÊM BIẾN LƯU TRỮ IMU
# actual_yaw = 0.0
# actual_gyroz = 0.0

# # Initialize empty data for plotting
# for _ in range(MAX_POINTS):
#     time_data.append(0)
#     tar_r_data.append(0)
#     act_r_data.append(0)
#     tar_l_data.append(0)
#     act_l_data.append(0)

# # ================= SERIAL READ THREAD =================
# def read_serial_thread():
#     global actual_vr, actual_vl, actual_yaw, actual_gyroz, running

#     while running and ser and ser.is_open:
#         try:
#             if ser.in_waiting:
#                 line = ser.readline().decode('utf-8', errors='ignore').strip()

#                 # STM32 giờ gửi 8 phần tử: FB, vR, vL, qX, qY, qZ, qW, gyroZ
#                 if line.startswith("FB,"):
#                     parts = line.split(',')

#                     if len(parts) >= 8: # Chặn lỗi index nếu mảng bị ngắn
#                         # 1. Đọc bánh xe
#                         actual_vr = float(parts[1])
#                         actual_vl = float(parts[2])

#                         # 2. Đọc Quaternion
#                         qx = float(parts[3])
#                         qy = float(parts[4])
#                         qz = float(parts[5])
#                         qw = float(parts[6])
                        
#                         # 3. Đọc Gyro Z
#                         actual_gyroz = float(parts[7])

#                         # DỊCH QUATERNION SANG YAW (ĐỘ)
#                         # Công thức: Yaw = atan2(2(qw*qz + qx*qy), 1 - 2(qy^2 + qz^2))
#                         siny_cosp = 2.0 * (qw * qz + qx * qy)
#                         cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
#                         yaw_rad = math.atan2(siny_cosp, cosy_cosp)
                        
#                         actual_yaw = math.degrees(yaw_rad) # Đổi ra Độ cho dễ nhìn

#         except Exception:
#             pass

# # ================= MAIN GUI =================
# root = tk.Tk()
# root.title("GUI Tuning PID & BNO055 IMU Viewer")
# root.geometry("1000x850") # Kéo dài cửa sổ ra chút để nhét bảng IMU

# # --- Connection Frame ---
# frame_top = tk.Frame(root, pady=10)
# frame_top.pack(side=tk.TOP, fill=tk.X)

# tk.Label(frame_top, text="UART:", font=('Arial', 12, 'bold')).pack(side=tk.LEFT, padx=10)
# tk.Label(frame_top, text=SERIAL_PORT, font=('Arial', 12)).pack(side=tk.LEFT)

# def connect_serial():
#     global ser, running, start_time

#     try:
#         ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
#         ser.setDTR(False)
#         ser.setRTS(False)

#         running = True
#         start_time = time.time()
#         threading.Thread(target=read_serial_thread, daemon=True).start()

#         btn_connect.config(text="Connected", state=tk.DISABLED, bg="green", fg="white")
#         root.after(50, send_and_update)

#     except Exception as e:
#         messagebox.showerror("Error", str(e))

# btn_connect = tk.Button(frame_top, text="Connect", font=('Arial', 10, 'bold'), bg="#3b82f6", fg="white", command=connect_serial)
# btn_connect.pack(side=tk.LEFT, padx=20)

# # --- Send Velocity Command & Update GUI ---
# def send_and_update():
#     if running and ser and ser.is_open:
#         vr = slider_r.get()
#         vl = slider_l.get()

#         cmd = f"CMD,{vr:.3f},{vl:.3f}\n"
#         try:
#             ser.write(cmd.encode())
#         except:
#             pass

#         # CẬP NHẬT ĐỒ THỊ BÁNH XE
#         current_t = time.time() - start_time
#         time_data.append(current_t)
#         tar_r_data.append(vr)
#         act_r_data.append(actual_vr)
#         tar_l_data.append(vl)
#         act_l_data.append(actual_vl)

#         # CẬP NHẬT CHỮ LÊN BẢNG IMU
#         lbl_yaw.config(text=f"🧭 Yaw Angle: {actual_yaw:7.2f}°")
#         lbl_gyroz.config(text=f"🌪️ Gyro Z: {actual_gyroz:7.2f} rad/s")

#         root.after(50, send_and_update)

# def stop_motors():
#     slider_r.set(0.0)
#     slider_l.set(0.0)

# def go_straight():
#     slider_r.set(0.3)
#     slider_l.set(0.3)

# # --- Plot Frame ---
# fig, (axR, axL) = plt.subplots(2, 1, figsize=(10, 4.5))
# fig.tight_layout(pad=4.0)

# line_tar_r, = axR.plot(time_data, tar_r_data, 'r--', label='Target')
# line_act_r, = axR.plot(time_data, act_r_data, 'b-', label='Actual')
# axR.set_title("Right wheel (m/s)", fontweight='bold')
# axR.set_ylabel("Velocity")
# axR.set_ylim(-0.6, 0.6)
# axR.legend(loc="upper right")
# axR.grid(True)

# line_tar_l, = axL.plot(time_data, tar_l_data, 'r--', label='Target')
# line_act_l, = axL.plot(time_data, act_l_data, 'b-', label='Actual')
# axL.set_title("Left wheel (m/s)", fontweight='bold')
# axL.set_ylabel("Velocity")
# axL.set_ylim(-0.6, 0.6)
# axL.legend(loc="upper right")
# axL.grid(True)

# canvas = FigureCanvasTkAgg(fig, master=root)
# canvas.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=1)

# def animate(i):
#     if not running: return
#     axR.set_xlim(time_data[0], time_data[-1])
#     axL.set_xlim(time_data[0], time_data[-1])
#     line_tar_r.set_data(time_data, tar_r_data)
#     line_act_r.set_data(time_data, act_r_data)
#     line_tar_l.set_data(time_data, tar_l_data)
#     line_act_l.set_data(time_data, act_l_data)
#     return line_tar_r, line_act_r, line_tar_l, line_act_l

# ani = animation.FuncAnimation(fig, animate, interval=100, blit=False)

# # ================= CONTROL PANEL =================
# control_frame = tk.Frame(root, pady=10, bg="#f1f5f9")
# control_frame.pack(side=tk.BOTTOM, fill=tk.BOTH, expand=True)

# # --- 🚀 THÊM BẢNG HIỂN THỊ IMU Ở ĐÂY ---
# imu_frame = tk.LabelFrame(control_frame, text=" BNO055 IMU DASHBOARD ", font=('Arial', 11, 'bold'), bg="#e0f2fe", fg="#0369a1")
# imu_frame.pack(side=tk.TOP, pady=5, padx=20, fill=tk.X)

# lbl_yaw = tk.Label(imu_frame, text="🧭 Yaw Angle:   0.00°", font=('Consolas', 18, 'bold'), fg="#1d4ed8", bg="#e0f2fe")
# lbl_yaw.pack(side=tk.LEFT, padx=50, pady=10)

# lbl_gyroz = tk.Label(imu_frame, text="🌪️ Gyro Z:   0.00 rad/s", font=('Consolas', 18, 'bold'), fg="#b91c1c", bg="#e0f2fe")
# lbl_gyroz.pack(side=tk.RIGHT, padx=50, pady=10)

# # --- PID Settings ---
# pid_frame = tk.LabelFrame(control_frame, text="Tuning PID parameter", font=('Arial', 10, 'bold'), bg="#f1f5f9")
# pid_frame.pack(side=tk.TOP, pady=5, padx=20)

# tk.Label(pid_frame, text="Kp:", font=('Arial', 12), bg="#f1f5f9").grid(row=0, column=0, padx=5, pady=5)
# entry_kp = tk.Entry(pid_frame, font=('Arial', 12), width=8, justify='center')
# entry_kp.insert(0, "600.0")
# entry_kp.grid(row=0, column=1, padx=5)

# tk.Label(pid_frame, text="Ki:", font=('Arial', 12), bg="#f1f5f9").grid(row=0, column=2, padx=5)
# entry_ki = tk.Entry(pid_frame, font=('Arial', 12), width=8, justify='center')
# entry_ki.insert(0, "80.0")
# entry_ki.grid(row=0, column=3, padx=5)

# def send_new_pid():
#     if running and ser and ser.is_open:
#         try:
#             kp_val = float(entry_kp.get())
#             ki_val = float(entry_ki.get())
#             cmd = f"PID,{kp_val:.2f},{ki_val:.2f}\n"
#             ser.write(cmd.encode())
#             messagebox.showinfo("Success", f"PID parameter downloaded to STM32!:\nKp = {kp_val}\nKi = {ki_val}")
#         except ValueError:
#             messagebox.showerror("Input Error", "Please enter valid numbers")

# tk.Button(pid_frame, text="Send PID parameter", bg="#f59e0b", fg="white", font=('Arial', 10, 'bold'), command=send_new_pid).grid(row=0, column=4, padx=15)

# # --- MOVEMENT CONTROLS ---
# movement_frame = tk.Frame(control_frame, bg="#f1f5f9")
# movement_frame.pack(side=tk.TOP, pady=5)

# btn_straight = tk.Button(movement_frame, text="⬆️ GO STRAIGHT (0.3 m/s)", bg="#10b981", fg="white", font=('Arial', 14, 'bold'), command=go_straight)
# btn_straight.pack(side=tk.LEFT, padx=15)

# btn_stop = tk.Button(movement_frame, text="🛑 EMERGENCY STOP (0 m/s)", bg="#ef4444", fg="white", font=('Arial', 14, 'bold'), command=stop_motors)
# btn_stop.pack(side=tk.LEFT, padx=15)

# # --- SLIDERS ---
# sliders_div = tk.Frame(control_frame, bg="#f1f5f9")
# sliders_div.pack(side=tk.TOP, fill=tk.X, pady=5)

# frame_l = tk.Frame(sliders_div, bg="#f1f5f9")
# frame_l.pack(side=tk.LEFT, expand=True)
# tk.Label(frame_l, text="Left Velocity:", font=('Arial', 12, 'bold'), bg="#f1f5f9").pack(side=tk.LEFT)
# slider_l = tk.Scale(frame_l, from_=-0.5, to=0.5, resolution=0.01, orient=tk.HORIZONTAL, length=250)
# slider_l.pack(side=tk.LEFT, padx=10)

# frame_r = tk.Frame(sliders_div, bg="#f1f5f9")
# frame_r.pack(side=tk.RIGHT, expand=True)
# tk.Label(frame_r, text="Right Velocity:", font=('Arial', 12, 'bold'), bg="#f1f5f9").pack(side=tk.LEFT)
# slider_r = tk.Scale(frame_r, from_=-0.5, to=0.5, resolution=0.01, orient=tk.HORIZONTAL, length=250)
# slider_r.pack(side=tk.LEFT, padx=10)

# # ================= CLOSE PROGRAM =================
# def on_closing():
#     global running
#     running = False
#     try:
#         if ser and ser.is_open:
#             ser.write(b"CMD,0.000,0.000\n")
#             time.sleep(0.1)
#             ser.close()
#     except:
#         pass
#     root.quit()
#     root.destroy()

# root.protocol("WM_DELETE_WINDOW", on_closing)

# if __name__ == "__main__":
#     root.mainloop()
import tkinter as tk
from tkinter import ttk, messagebox
import serial
import threading
import time
import collections
import math # Thư viện tính toán góc

import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

# ================= GLOBAL VARIABLES ==================
BAUD_RATE = 115200
SERIAL_PORT = "/dev/serial0" # Chạy trực tiếp trên Raspberry Pi
MAX_POINTS = 100

running = False
ser = None

time_data = collections.deque(maxlen=MAX_POINTS)
tar_r_data = collections.deque(maxlen=MAX_POINTS)
act_r_data = collections.deque(maxlen=MAX_POINTS)
tar_l_data = collections.deque(maxlen=MAX_POINTS)
act_l_data = collections.deque(maxlen=MAX_POINTS)

start_time = 0
actual_vr = 0.0
actual_vl = 0.0

# THÊM BIẾN LƯU TRỮ GÓC YAW (THETA)
actual_yaw_rad = 0.0
actual_yaw_deg = 0.0

# Initialize empty data for plotting
for _ in range(MAX_POINTS):
    time_data.append(0)
    tar_r_data.append(0)
    act_r_data.append(0)
    tar_l_data.append(0)
    act_l_data.append(0)

# ================= SERIAL READ THREAD =================
def read_serial_thread():
    global actual_vr, actual_vl, actual_yaw_rad, actual_yaw_deg, running

    while running and ser and ser.is_open:
        try:
            if ser.in_waiting:
                line = ser.readline().decode('utf-8', errors='ignore').strip()

                # STM32 giờ gửi 4 phần tử siêu gọn: FB, vR, vL, theta
                if line.startswith("FB,"):
                    parts = line.split(',')

                    if len(parts) >= 4: 
                        # 1. Đọc bánh xe
                        actual_vr = float(parts[1])
                        actual_vl = float(parts[2])

                        # 2. Đọc góc Theta (Radian) trực tiếp từ STM32
                        actual_yaw_rad = float(parts[3])
                        
                        # Đổi ra Độ (Degree) để hiển thị lên bảng GUI cho trực quan
                        actual_yaw_deg = math.degrees(actual_yaw_rad)

        except Exception:
            pass

# ================= MAIN GUI =================
root = tk.Tk()
root.title("GUI Tuning PID & BNO055 IMU Viewer")
root.geometry("1000x850") 

# --- Connection Frame ---
frame_top = tk.Frame(root, pady=10)
frame_top.pack(side=tk.TOP, fill=tk.X)

tk.Label(frame_top, text="UART:", font=('Arial', 12, 'bold')).pack(side=tk.LEFT, padx=10)
tk.Label(frame_top, text=SERIAL_PORT, font=('Arial', 12)).pack(side=tk.LEFT)

def connect_serial():
    global ser, running, start_time

    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
        ser.setDTR(False)
        ser.setRTS(False)

        running = True
        start_time = time.time()
        threading.Thread(target=read_serial_thread, daemon=True).start()

        btn_connect.config(text="Connected", state=tk.DISABLED, bg="green", fg="white")
        root.after(50, send_and_update)

    except Exception as e:
        messagebox.showerror("Error", str(e))

btn_connect = tk.Button(frame_top, text="Connect", font=('Arial', 10, 'bold'), bg="#3b82f6", fg="white", command=connect_serial)
btn_connect.pack(side=tk.LEFT, padx=20)

# --- Send Velocity Command & Update GUI ---
def send_and_update():
    if running and ser and ser.is_open:
        vr = slider_r.get()
        vl = slider_l.get()

        cmd = f"CMD,{vr:.3f},{vl:.3f}\n"
        try:
            ser.write(cmd.encode())
        except:
            pass

        # CẬP NHẬT ĐỒ THỊ BÁNH XE
        current_t = time.time() - start_time
        time_data.append(current_t)
        tar_r_data.append(vr)
        act_r_data.append(actual_vr)
        tar_l_data.append(vl)
        act_l_data.append(actual_vl)

        # CẬP NHẬT CHỮ LÊN BẢNG IMU
        lbl_yaw_deg.config(text=f"🧭 Yaw (Degree): {actual_yaw_deg:7.2f}°")
        lbl_yaw_rad.config(text=f"📐 Theta (Radian): {actual_yaw_rad:7.3f} rad")

        root.after(50, send_and_update)

def stop_motors():
    slider_r.set(0.0)
    slider_l.set(0.0)

def go_straight():
    slider_r.set(0.3)
    slider_l.set(0.3)

# --- Plot Frame ---
fig, (axR, axL) = plt.subplots(2, 1, figsize=(10, 4.5))
fig.tight_layout(pad=4.0)

line_tar_r, = axR.plot(time_data, tar_r_data, 'r--', label='Target')
line_act_r, = axR.plot(time_data, act_r_data, 'b-', label='Actual')
axR.set_title("Right wheel (m/s)", fontweight='bold')
axR.set_ylabel("Velocity")
axR.set_ylim(-0.6, 0.6)
axR.legend(loc="upper right")
axR.grid(True)

line_tar_l, = axL.plot(time_data, tar_l_data, 'r--', label='Target')
line_act_l, = axL.plot(time_data, act_l_data, 'b-', label='Actual')
axL.set_title("Left wheel (m/s)", fontweight='bold')
axL.set_ylabel("Velocity")
axL.set_ylim(-0.6, 0.6)
axL.legend(loc="upper right")
axL.grid(True)

canvas = FigureCanvasTkAgg(fig, master=root)
canvas.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=1)

def animate(i):
    if not running: return
    axR.set_xlim(time_data[0], time_data[-1])
    axL.set_xlim(time_data[0], time_data[-1])
    line_tar_r.set_data(time_data, tar_r_data)
    line_act_r.set_data(time_data, act_r_data)
    line_tar_l.set_data(time_data, tar_l_data)
    line_act_l.set_data(time_data, act_l_data)
    return line_tar_r, line_act_r, line_tar_l, line_act_l

ani = animation.FuncAnimation(fig, animate, interval=100, blit=False)

# ================= CONTROL PANEL =================
control_frame = tk.Frame(root, pady=10, bg="#f1f5f9")
control_frame.pack(side=tk.BOTTOM, fill=tk.BOTH, expand=True)

# --- 🚀 BẢNG HIỂN THỊ IMU ---
imu_frame = tk.LabelFrame(control_frame, text=" BNO055 HEADING DASHBOARD ", font=('Arial', 11, 'bold'), bg="#e0f2fe", fg="#0369a1")
imu_frame.pack(side=tk.TOP, pady=5, padx=20, fill=tk.X)

lbl_yaw_deg = tk.Label(imu_frame, text="🧭 Yaw (Degree):   0.00°", font=('Consolas', 18, 'bold'), fg="#1d4ed8", bg="#e0f2fe")
lbl_yaw_deg.pack(side=tk.LEFT, padx=50, pady=10)

lbl_yaw_rad = tk.Label(imu_frame, text="📐 Theta (Radian):   0.00 rad", font=('Consolas', 18, 'bold'), fg="#b91c1c", bg="#e0f2fe")
lbl_yaw_rad.pack(side=tk.RIGHT, padx=50, pady=10)

# --- PID Settings ---
pid_frame = tk.LabelFrame(control_frame, text="Tuning PID parameter", font=('Arial', 10, 'bold'), bg="#f1f5f9")
pid_frame.pack(side=tk.TOP, pady=5, padx=20)

tk.Label(pid_frame, text="Kp:", font=('Arial', 12), bg="#f1f5f9").grid(row=0, column=0, padx=5, pady=5)
entry_kp = tk.Entry(pid_frame, font=('Arial', 12), width=8, justify='center')
entry_kp.insert(0, "600.0")
entry_kp.grid(row=0, column=1, padx=5)

tk.Label(pid_frame, text="Ki:", font=('Arial', 12), bg="#f1f5f9").grid(row=0, column=2, padx=5)
entry_ki = tk.Entry(pid_frame, font=('Arial', 12), width=8, justify='center')
entry_ki.insert(0, "80.0")
entry_ki.grid(row=0, column=3, padx=5)

def send_new_pid():
    if running and ser and ser.is_open:
        try:
            kp_val = float(entry_kp.get())
            ki_val = float(entry_ki.get())
            cmd = f"PID,{kp_val:.2f},{ki_val:.2f}\n"
            ser.write(cmd.encode())
            messagebox.showinfo("Success", f"PID parameter downloaded to STM32!:\nKp = {kp_val}\nKi = {ki_val}")
        except ValueError:
            messagebox.showerror("Input Error", "Please enter valid numbers")

tk.Button(pid_frame, text="Send PID parameter", bg="#f59e0b", fg="white", font=('Arial', 10, 'bold'), command=send_new_pid).grid(row=0, column=4, padx=15)

# --- MOVEMENT CONTROLS ---
movement_frame = tk.Frame(control_frame, bg="#f1f5f9")
movement_frame.pack(side=tk.TOP, pady=5)

btn_straight = tk.Button(movement_frame, text="⬆️ GO STRAIGHT (0.3 m/s)", bg="#10b981", fg="white", font=('Arial', 14, 'bold'), command=go_straight)
btn_straight.pack(side=tk.LEFT, padx=15)

btn_stop = tk.Button(movement_frame, text="🛑 EMERGENCY STOP (0 m/s)", bg="#ef4444", fg="white", font=('Arial', 14, 'bold'), command=stop_motors)
btn_stop.pack(side=tk.LEFT, padx=15)

# --- SLIDERS ---
sliders_div = tk.Frame(control_frame, bg="#f1f5f9")
sliders_div.pack(side=tk.TOP, fill=tk.X, pady=5)

frame_l = tk.Frame(sliders_div, bg="#f1f5f9")
frame_l.pack(side=tk.LEFT, expand=True)
tk.Label(frame_l, text="Left Velocity:", font=('Arial', 12, 'bold'), bg="#f1f5f9").pack(side=tk.LEFT)
slider_l = tk.Scale(frame_l, from_=-0.5, to=0.5, resolution=0.01, orient=tk.HORIZONTAL, length=250)
slider_l.pack(side=tk.LEFT, padx=10)

frame_r = tk.Frame(sliders_div, bg="#f1f5f9")
frame_r.pack(side=tk.RIGHT, expand=True)
tk.Label(frame_r, text="Right Velocity:", font=('Arial', 12, 'bold'), bg="#f1f5f9").pack(side=tk.LEFT)
slider_r = tk.Scale(frame_r, from_=-0.5, to=0.5, resolution=0.01, orient=tk.HORIZONTAL, length=250)
slider_r.pack(side=tk.LEFT, padx=10)

# ================= CLOSE PROGRAM =================
def on_closing():
    global running
    running = False
    try:
        if ser and ser.is_open:
            ser.write(b"CMD,0.000,0.000\n")
            time.sleep(0.1)
            ser.close()
    except:
        pass
    root.quit()
    root.destroy()

root.protocol("WM_DELETE_WINDOW", on_closing)

if __name__ == "__main__":
    root.mainloop()