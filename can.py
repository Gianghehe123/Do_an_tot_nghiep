import serial
import struct
import time
import threading
import sys
from collections import deque
import matplotlib.pyplot as plt
import matplotlib.animation as animation

PORT = "COM6"
BAUDRATE = 115200

try:
    ser = serial.Serial(PORT, BAUDRATE, timeout=0.1)
except Exception as e:
    print(f"Loi khong mo duoc cong {PORT}: {e}")
    sys.exit(1)

ser.write(b"O\r")
time.sleep(0.1)

is_running = True
latest_state = 0
latest_err = 0
latest_rpm = 0.0
latest_vq = 0.0

latest_iq = 0.0
latest_te = 0.0
latest_tau_ext = 0.0
latest_col_status = 0  # 0: Binh thuong, 1: Dang cham/giu tay, 2: Da tha (dem nguoc 3s)
latest_countdown = 0.0

current_control_mode = 0  # 0: SPEED MODE, 1: DUTY MODE
current_target_val = 0.0

STATE_NAMES = {0: "IDLE (Dung)", 1: "ALIGNING (Khoa goc can chinh)", 2: "RUNNING (Dang chay)", 3: "ERROR (Loi)"}

BUFFER_LEN = 300
buf_time         = deque(maxlen=BUFFER_LEN)
buf_actual_rpm   = deque(maxlen=BUFFER_LEN)
buf_target_rpm   = deque(maxlen=BUFFER_LEN)
buf_iq           = deque(maxlen=BUFFER_LEN)
buf_te           = deque(maxlen=BUFFER_LEN)
buf_tau_ext      = deque(maxlen=BUFFER_LEN)
buf_col_status   = deque(maxlen=BUFFER_LEN)

start_time = time.time()

def rx_listener():
    global latest_state, latest_err, latest_rpm, latest_vq
    global latest_iq, latest_te, latest_tau_ext, latest_col_status, latest_countdown
    buffer = ""
    while is_running:
        try:
            if ser.in_waiting > 0:
                raw_bytes = ser.read(ser.in_waiting)
                buffer += raw_bytes.decode('ascii', errors='ignore')
                
                while '\r' in buffer:
                    line, buffer = buffer.split('\r', 1)
                    line = line.strip()
                    
                    if line.startswith("t1028") and len(line) == 21:
                        data = bytes.fromhex(line[5:21])
                        latest_state = data[0]
                        latest_err   = data[1]
                        rpm_scaled, vq_scaled = struct.unpack('<hh', data[2:6])
                        latest_rpm = rpm_scaled / 10.0
                        latest_vq  = vq_scaled / 100.0
                    
                    elif line.startswith("t1038") and len(line) == 21:
                        data = bytes.fromhex(line[5:21])
                        iq_ma, te_mnm, tau_mnm, col_st, count_t = struct.unpack('<hhhBB', data[:8])
                        latest_iq         = iq_ma / 1000.0
                        latest_te         = te_mnm / 1000.0
                        latest_tau_ext    = tau_mnm / 1000.0
                        latest_col_status = col_st
                        latest_countdown  = count_t / 10.0

                        t_now = time.time() - start_time
                        buf_time.append(t_now)
                        buf_actual_rpm.append(latest_rpm)
                        buf_target_rpm.append(current_target_val if current_control_mode == 0 else 0.0)
                        buf_iq.append(latest_iq)
                        buf_te.append(latest_te)
                        buf_tau_ext.append(latest_tau_ext)
                        buf_col_status.append(latest_col_status)

        except Exception:
            pass
        time.sleep(0.002)

threading.Thread(target=rx_listener, daemon=True).start()

def show_live_plot():
    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(11, 9.0), sharex=True)
    fig.canvas.manager.set_window_title("ODrive 3.5 - FOC & GMO Light Touch Scope")

    # 1. Đồ thị Tốc độ
    line_target, = ax1.plot([], [], label="Target RPM", color="black", linestyle="--", linewidth=1.2)
    line_actual, = ax1.plot([], [], label="Actual RPM", color="purple", linewidth=1.5)
    ax1.set_ylabel("Speed (RPM)")
    ax1.grid(True, linestyle=":", alpha=0.6)
    ax1.legend(loc="upper right")
    ax1.set_title("1. Rotor Velocity (Target vs Actual)")

    # 2. Đồ thị Iq & Te
    line_iq, = ax2.plot([], [], label="Iq (A)", color="navy", linewidth=1.3)
    line_te, = ax2.plot([], [], label="Te - Motor Torque (Nm)", color="crimson", linewidth=1.5)
    ax2.axhline(0.0, color="gray", linestyle=":", alpha=0.6)
    ax2.set_ylabel("Torque (Nm) / Iq (A)")
    ax2.grid(True, linestyle=":", alpha=0.6)
    ax2.legend(loc="upper right")
    ax2.set_title("2. Motor Torque Te = Kt * Iq (Kt = 0.0955 Nm/A)")

    # 3. Đồ thị GMO Va chạm
    line_tau, = ax3.plot([], [], label="External Torque (GMO)", color="darkorange", linewidth=1.5)
    line_hit, = ax3.plot([], [], 'ro', markersize=6, label="TOUCH / COLLISION POINT")
    ax3.axhline(0.025, color="red", linestyle="--", alpha=0.8, label="Touch Threshold (+0.025 Nm)")
    ax3.axhline(-0.025, color="red", linestyle="--", alpha=0.8, label="Touch Threshold (-0.025 Nm)")
    ax3.set_xlabel("Time (s)")
    ax3.set_ylabel("Disturbance (Nm)")
    ax3.grid(True, linestyle=":", alpha=0.6)
    ax3.legend(loc="upper right")
    ax3.set_title("3. Collision Detection (Light Touch Sensitivity: 0.025 Nm)")

    def animate(frame):
        if len(buf_time) < 2:
            return line_target, line_actual, line_iq, line_te, line_tau, line_hit

        t = list(buf_time)
        ax1.set_xlim(t[0], t[-1])

        # Subplot 1
        line_target.set_data(t, list(buf_target_rpm))
        line_actual.set_data(t, list(buf_actual_rpm))
        all_rpm = list(buf_actual_rpm) + list(buf_target_rpm)
        if all_rpm:
            max_r = max(max(all_rpm), abs(min(all_rpm)), 20.0)
            ax1.set_ylim(-max_r * 1.2 if min(all_rpm) < -5 else -10, max_r * 1.2)

        # Subplot 2
        line_iq.set_data(t, list(buf_iq))
        line_te.set_data(t, list(buf_te))
        all_t2 = list(buf_iq) + list(buf_te)
        if all_t2:
            max_c = max(max(all_t2), abs(min(all_t2)), 0.2)
            ax2.set_ylim(-max_c * 1.3, max_c * 1.3)

        # Subplot 3
        tau_list = list(buf_tau_ext)
        st_list  = list(buf_col_status)
        line_tau.set_data(t, tau_list)

        hit_t   = [t[i] for i in range(len(t)) if st_list[i] == 1]
        hit_tau = [tau_list[i] for i in range(len(t)) if st_list[i] == 1]
        line_hit.set_data(hit_t, hit_tau)

        if tau_list:
            max_tau = max(max(tau_list), abs(min(tau_list)), 0.06)
            ax3.set_ylim(-max_tau * 1.4, max_tau * 1.4)

        # Đổi màu nền theo tình trạng
        if latest_col_status == 1:
            ax3.set_facecolor('#FFE0E0') # Đang chạm/giữ tay (Đỏ nhạt)
            ax3.set_title(f"3. TOUCH DETECTED! MOTOR STOPPED (Limit <= 0.1A)")
        elif latest_col_status == 2:
            ax3.set_facecolor('#FFF8DC') # Đã thả, đang chờ 3s (Vàng nhạt)
            ax3.set_title(f"3. RELEASED! RESUMING IN {latest_countdown:.1f}s...")
        else:
            ax3.set_facecolor('white')
            ax3.set_title("3. Collision Detection (Light Touch Sensitivity: 0.025 Nm)")

        return line_target, line_actual, line_iq, line_te, line_tau, line_hit

    ani = animation.FuncAnimation(fig, animate, interval=50, blit=False)
    plt.tight_layout()
    print("\n[OK] Đang mở Scope... (Đóng cửa sổ đồ thị để quay lại Menu)")
    plt.show()

def print_menu():
    mode_str = "DUTY MODE (%)" if current_control_mode == 1 else "SPEED MODE (RPM)"
    unit_str = "%" if current_control_mode == 1 else "RPM"

    # Cực tính Iq
    if abs(latest_rpm) < 1.0:
        polarity_str = "DUNG YEN (~0A)"
    elif (latest_rpm > 0 and latest_iq > 0) or (latest_rpm < 0 and latest_iq < 0):
        polarity_str = "THUAN (+) [Cung chieu quay - Dang sinh cong keo]"
    else:
        polarity_str = "NGHICH (-) [Nguoc chieu - Dang bi ngoai luc can]"

    # Tình trạng va chạm & đếm ngược
    if latest_col_status == 1:
        col_str = ">> [DUNG AN TOAN]: DANG CHAM/GIU TAY (Dong an <= 0.1A) <<"
    elif latest_col_status == 2:
        col_str = f">> [DA THA TAY]: CHO 3s DE CHAY LAI (Con lai: {latest_countdown:.1f}s) <<"
    else:
        col_str = "Binh thuong (Khong cham)"
    
    print("\n================= ODRIVE 3.5 - FOC & GMO LIGHT TOUCH OBSERVER =================")
    print(f" [CHE DO]: >>> {mode_str} <<<  |  [TRANG THAI]: {STATE_NAMES.get(latest_state, 'Unknown')}")
    print("-------------------------------------------------------------------------------")
    print(f" Toc do thuc te      : {latest_rpm:6.1f} RPM  |  Dien ap Vq: {latest_vq:5.2f} V")
    print(f" Dong dien mo-men Iq : {latest_iq:+6.3f} A")
    print(f" CUC TINH IQ         : >>> {polarity_str} <<<")
    print(f" MO-MEN DONG CO (Te) : {latest_te:+6.3f} Nm  (Te = Kt * Iq)")
    print(f" MO-MEN NGOAI LUC (r): {latest_tau_ext:+6.3f} Nm  (GMO Disturbance)")
    print(f" TINH TRANG VA CHAM  : >>> {col_str} <<<")
    print("-------------------------------------------------------------------------------")
    print(" [1] DUNG dong co (Stop / Disarm)")
    print(" [2] CHAY dong co (Run / Arm & Auto Align)")
    print(f" [3] SET TARGET (Muc tieu: {current_target_val:.1f} {unit_str})")
    print(" [4] XEM CHAN DOAN CHI TIET (Diagnostics)")
    print(f" [5] DOI CHE DO (Chuyen sang {'SPEED MODE' if current_control_mode == 1 else 'DUTY MODE'})")
    print(" [6] CALIB LAI DIEM 0 SHUNT")
    print(" [7] XEM BIEU DO REAL-TIME (Scope RPM, Te & Va cham GMO)")
    print(" [q] Thoat chuong trinh")
    print("===============================================================================")
    print("Chon lenh: ", end="", flush=True)

print_menu()

try:
    while True:
        choice = input().strip()
        if choice == '1':
            ser.write(b"t100101\r")
            print("[OK] Da gui lenh DUNG.")
            time.sleep(0.1)
            print_menu()
        elif choice == '2':
            ser.write(b"t100102\r")
            print("[OK] Da gui lenh CHAY...")
            time.sleep(0.1)
            print_menu()
        elif choice == '3':
            try:
                unit_str = "%" if current_control_mode == 1 else "RPM"
                target = float(input(f">> Nhap gia tri ({unit_str}): "))
                current_target_val = target
                payload_bytes = struct.pack('<f', target)
                ser.write(f"t100503{payload_bytes.hex().upper()}\r".encode('ascii'))
                print(f"[OK] Da cap nhat: {target}")
            except ValueError:
                print("[LOI] Gia tri khong hop le!")
            time.sleep(0.1)
            print_menu()
        elif choice == '4':
            ser.write(b"t100104\r")
            time.sleep(0.1)
            print("\n---------------- KET QUA CHAN DOAN GMO ----------------")
            print(f" Trang thai        : {STATE_NAMES.get(latest_state, 'Unknown')}")
            print(f" Toc do thuc te    : {latest_rpm:.1f} RPM")
            print(f" Dong mo-men Iq    : {latest_iq:.3f} A")
            print(f" Mo-men dien tu Te : {latest_te:.4f} Nm")
            print(f" Mo-men va cham r  : {latest_tau_ext:.4f} Nm")
            print(f" Trang thai cham   : {latest_col_status} (0: BT, 1: Giu tay, 2: Dem nguoc)")
            print("-------------------------------------------------------")
            print_menu()
        elif choice == '5':
            current_control_mode = 1 if current_control_mode == 0 else 0
            current_target_val = 0.0
            mode_val = 1.0 if current_control_mode == 1 else 0.0
            payload_bytes = struct.pack('<f', mode_val)
            ser.write(f"t100505{payload_bytes.hex().upper()}\r".encode('ascii'))
            print(f"\n[OK] Da chuyen sang {'DUTY MODE' if current_control_mode == 1 else 'SPEED MODE'}!")
            time.sleep(0.1)
            print_menu()
        elif choice == '6':
            ser.write(b"t100106\r")
            print("\n[OK] Dang calib lai offset...")
            time.sleep(0.7)
            print_menu()
        elif choice == '7':
            show_live_plot()
            print_menu()
        elif choice.lower() == 'q':
            ser.write(b"t100101\r")
            time.sleep(0.05)
            ser.write(b"C\r")
            ser.close()
            break
        else:
            print_menu()
except KeyboardInterrupt:
    ser.write(b"t100101\r")
    ser.close()