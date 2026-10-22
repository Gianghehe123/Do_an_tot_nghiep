import serial
import struct
import time
import threading
import sys
import msvcrt
from collections import deque
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np

# Vô hiệu hóa các phím tắt mặc định của Matplotlib trùng với phím điều khiển ('s', 'f', 'l')
plt.rcParams['keymap.save'] = []
plt.rcParams['keymap.fullscreen'] = []
plt.rcParams['keymap.yscale'] = []
plt.rcParams['keymap.xscale'] = []

PORT = "COM6"
BAUDRATE = 115200

try:
    ser = serial.Serial(PORT, BAUDRATE, timeout=0.05)
except Exception as e:
    print(f"[LOI] Khong the mo cong {PORT}: {e}")
    sys.exit(1)

# Giữ nguyên chuỗi khởi tạo USB-CAN y hệt file can.py gốc của bạn
ser.write(b"C\r")
time.sleep(0.05)
ser.write(b"O\r")
time.sleep(0.1)

is_running = True
latest_state = 0
is_lut_active = 1
is_fw_active = 0
is_torque_mode = 0  # 0: SPEED Mode, 1: TORQUE Mode

# Giá trị đặt
current_target_rpm = 0.0
current_target_iq  = 0.00

# Đo lường
latest_rpm = 0.0
latest_vq = 0.0
latest_raw_counts = 0
latest_ia = 0.0
latest_ib = 0.0
latest_ic = 0.0
latest_id = 0.0
latest_iq = 0.0

# GMO
latest_tau_ext = 0.0
latest_tau_thresh = 0.035
is_collided = 0
countdown_sec = 0

STATE_SHORT = {0: "IDLE", 1: "CALIB", 2: "RUN", 3: "COLLISION", 4: "ERR"}

BUFFER_LEN = 250
start_time = time.time()
buf_time       = deque([0.0, 0.025], maxlen=BUFFER_LEN)
buf_rpm        = deque([0.0, 0.0],   maxlen=BUFFER_LEN)
buf_rpm_ref    = deque([0.0, 0.0],   maxlen=BUFFER_LEN)
buf_iq         = deque([0.0, 0.0],   maxlen=BUFFER_LEN)
buf_iq_ref     = deque([0.0, 0.0],   maxlen=BUFFER_LEN)
buf_id         = deque([0.0, 0.0],   maxlen=BUFFER_LEN)
buf_tau_ext    = deque([0.0, 0.0],   maxlen=BUFFER_LEN)
buf_tau_thresh = deque([0.035, 0.035], maxlen=BUFFER_LEN)

received_lut = np.zeros(128)
received_mask = np.zeros(128, dtype=bool)


def send_can_cmd(cmd_id, payload=0.0):
    data = bytearray(8)
    data[0] = cmd_id
    data[1:5] = struct.pack('<f', float(payload))
    hex_str = data.hex().upper()
    ser.write(f"t1008{hex_str}\r".encode('ascii'))
    ser.flush()


def rx_listener():
    global latest_state, is_lut_active, is_fw_active, is_torque_mode
    global latest_rpm, latest_vq, latest_raw_counts
    global latest_ia, latest_ib, latest_ic, latest_id, latest_iq
    global latest_tau_ext, latest_tau_thresh, is_collided, countdown_sec

    buffer = ""
    while is_running:
        try:
            if ser.in_waiting > 0:
                raw_bytes = ser.read(ser.in_waiting)
                buffer += raw_bytes.decode('ascii', errors='ignore')

                while '\r' in buffer:
                    line, buffer = buffer.split('\r', 1)
                    line = line.strip()

                    # 0x102: Status
                    if line.startswith("t1028") and len(line) == 21:
                        data = bytes.fromhex(line[5:21])
                        latest_state   = data[0]
                        flags          = data[1]
                        is_lut_active  = 1 if (flags & 0x01) else 0
                        is_fw_active   = 1 if (flags & 0x02) else 0
                        is_torque_mode = 1 if (flags & 0x04) else 0

                        rpm_scaled, vq_scaled, raw_counts = struct.unpack('<hhH', data[2:8])
                        latest_rpm = rpm_scaled / 10.0
                        latest_vq  = vq_scaled / 100.0
                        latest_raw_counts = raw_counts & 0x3FFF

                    # 0x105: Dòng điện (Ia, Ib, Id, Iq)
                    elif line.startswith("t1058") and len(line) == 21:
                        data = bytes.fromhex(line[5:21])
                        ia_ma, ib_ma, id_ma, iq_ma = struct.unpack('<hhhh', data[0:8])
                        latest_ia = ia_ma / 1000.0
                        latest_ib = ib_ma / 1000.0
                        latest_ic = -(latest_ia + latest_ib)
                        latest_id = id_ma / 1000.0
                        latest_iq = iq_ma / 1000.0

                        t_now = time.time() - start_time
                        buf_time.append(t_now)
                        buf_rpm.append(latest_rpm)
                        buf_rpm_ref.append(current_target_rpm if is_torque_mode == 0 else latest_rpm)
                        buf_iq.append(latest_iq)
                        buf_iq_ref.append(current_target_iq if is_torque_mode == 1 else 0.0)
                        buf_id.append(latest_id)
                        buf_tau_ext.append(latest_tau_ext)
                        buf_tau_thresh.append(latest_tau_thresh)

                    # 0x106: GMO Telemetry
                    elif line.startswith("t1068") and len(line) == 21:
                        data = bytes.fromhex(line[5:21])
                        tau_mnm, thresh_mnm = struct.unpack('<hh', data[0:4])
                        latest_tau_ext    = tau_mnm / 1000.0
                        latest_tau_thresh = thresh_mnm / 1000.0
                        is_collided       = data[4]
                        countdown_sec     = data[5]

                    # 0x104: Bảng LUT
                    elif line.startswith("t1048") and len(line) == 21:
                        data = bytes.fromhex(line[5:21])
                        idx = data[0]
                        if idx < 128:
                            val = struct.unpack('<f', data[1:5])[0]
                            received_lut[idx] = val
                            received_mask[idx] = True

        except Exception:
            pass
        time.sleep(0.001)


threading.Thread(target=rx_listener, daemon=True).start()


def process_key(ch):
    global current_target_iq, current_target_rpm, is_fw_active, is_lut_active, is_torque_mode
    if ch in ['+', '=']:
        if is_torque_mode == 1:
            current_target_iq = round(current_target_iq + 0.05, 2)
            send_can_cmd(0x03, current_target_iq)
        else:
            current_target_rpm += 10.0
            send_can_cmd(0x07, current_target_rpm)
    elif ch in ['-', '_']:
        if is_torque_mode == 1:
            current_target_iq = round(current_target_iq - 0.05, 2)
            send_can_cmd(0x03, current_target_iq)
        else:
            current_target_rpm -= 10.0
            send_can_cmd(0x07, current_target_rpm)
    elif ch in ['t', 'T']:
        if is_torque_mode == 1:
            is_torque_mode = 0
            current_target_rpm = float(int(latest_rpm))
            send_can_cmd(0x07, current_target_rpm)
        else:
            is_torque_mode = 1
            current_target_iq = round(latest_iq, 2)
            send_can_cmd(0x03, current_target_iq)
    elif ch in ['a', 'A']:
        send_can_cmd(0x02)
    elif ch in [' ', 's', 'S', '0']:
        current_target_iq = 0.00
        current_target_rpm = 0.0
        send_can_cmd(0x01)
    elif ch in ['f', 'F']:
        new_fw = 0 if is_fw_active == 1 else 1
        send_can_cmd(0x06, float(new_fw))
    elif ch in ['m', 'M']:
        new_mode = 0 if is_lut_active == 1 else 1
        send_can_cmd(0x05, float(new_mode))
    elif ch in ['c', 'C']:
        received_mask.fill(False)
        send_can_cmd(0x04)


def show_foc_gmo_scope():
    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(11, 9.5), sharex=True)
    fig.canvas.manager.set_window_title("20kHz MIN-MAX FOC + GMO COLLISION DETECTION (GIM8108)")

    # BẢNG 1: RPM
    line_rpm,     = ax1.plot(list(buf_time), list(buf_rpm), color="tab:blue", linewidth=1.8, label="RPM Thực Tế")
    line_rpm_ref, = ax1.plot(list(buf_time), list(buf_rpm_ref), color="darkorange", linestyle="--", linewidth=1.5, label="RPM Target")
    ax1.axhline(0, color="black", linestyle="--", alpha=0.4, linewidth=1.0)
    ax1.set_ylabel("Vận Tốc (RPM)", fontsize=9)
    ax1.grid(True, linestyle=":")
    ax1.legend(loc="upper right")
    ax1.set_title("1. Bảng Vận Tốc Rotor (RPM Scope - 2 kHz Loop)", fontweight="bold", fontsize=10)

    # BẢNG 2: DÒNG ĐIỆN IQ, ID
    line_iq,     = ax2.plot(list(buf_time), list(buf_iq), color="darkblue", linewidth=1.8, label="Iq Thực Tế (Mô-men)")
    line_iq_ref, = ax2.plot(list(buf_time), list(buf_iq_ref), color="orange", linestyle="--", linewidth=1.5, label="Iq Target")
    line_id,     = ax2.plot(list(buf_time), list(buf_id), color="crimson", linewidth=1.8, label="Id Thực Tế (Từ Thông/FW)")
    ax2.axhline(0, color="black", linestyle="--", alpha=0.4, linewidth=1.0)
    ax2.set_ylabel("Dòng D-Q (A)", fontsize=9)
    ax2.grid(True, linestyle=":")
    ax2.legend(loc="upper right")
    ax2.set_title("2. Bảng Dòng Điện Trực Giao Rotor (20 kHz FOC Loop)", fontweight="bold", fontsize=10)

    # BẢNG 3: BỘ QUAN SÁT GMO
    line_tau,   = ax3.plot(list(buf_time), list(buf_tau_ext), color="purple", linewidth=2.0, label="Tau_ext (Mô-men Ngoại Lực GMO)")
    line_th_up, = ax3.plot(list(buf_time), list(buf_tau_thresh), color="red", linestyle="--", linewidth=1.5, label="+Ngưỡng Va Chạm")
    line_th_dn, = ax3.plot(list(buf_time), [-x for x in buf_tau_thresh], color="red", linestyle="--", linewidth=1.5, label="-Ngưỡng Va Chạm")
    ax3.axhline(0, color="black", linestyle="--", alpha=0.4, linewidth=1.0)
    ax3.set_xlabel("Thời Gian (s)", fontsize=9)
    ax3.set_ylabel("Tau (N.m)", fontsize=9)
    ax3.grid(True, linestyle=":")
    ax3.legend(loc="upper right")
    ax3.set_title("3. Bảng Giám Sát Va Chạm (Generalized Momentum Observer - 1 kHz)", fontweight="bold", fontsize=10)

    def on_key_scope(event):
        if event.key is not None:
            process_key(event.key)

    fig.canvas.mpl_connect('key_press_event', on_key_scope)

    def animate(frame):
        # Đọc cả phím bấm từ cửa sổ Terminal đen ngay trong lúc Scope đang mở
        while msvcrt.kbhit():
            ch = msvcrt.getch().decode('ascii', errors='ignore')
            process_key(ch)

        t = list(buf_time)
        ax1.set_xlim(t[0], max(t[-1], t[0] + 3.0))

        # Cập nhật Bảng 1 (RPM)
        line_rpm.set_data(t, list(buf_rpm))
        line_rpm_ref.set_data(t, list(buf_rpm_ref))
        max_r = max(max(np.abs(list(buf_rpm))), abs(current_target_rpm), 20.0)
        ax1.set_ylim(-max_r * 1.3, max_r * 1.3)

        # Cập nhật Bảng 2 (Iq, Id)
        line_iq.set_data(t, list(buf_iq))
        line_iq_ref.set_data(t, list(buf_iq_ref))
        line_id.set_data(t, list(buf_id))
        all_dq = list(buf_iq) + list(buf_id) + [abs(current_target_iq)]
        if all_dq:
            max_dq = max(max(np.abs(all_dq)), 0.4)
            ax2.set_ylim(-max_dq * 1.35, max_dq * 1.35)

        # Cập nhật Bảng 3 (GMO)
        line_tau.set_data(t, list(buf_tau_ext))
        line_th_up.set_data(t, list(buf_tau_thresh))
        line_th_dn.set_data(t, [-x for x in buf_tau_thresh])
        max_tau = max(max(np.abs(list(buf_tau_ext))), latest_tau_thresh * 2.2, 0.06)
        ax3.set_ylim(-max_tau * 1.25, max_tau * 1.25)

        mode_str = "TORQUE MODE" if is_torque_mode == 1 else "SPEED MODE"
        fw_str   = "FW:ON" if is_fw_active == 1 else "FW:OFF"
        lut_str  = "LUT:ON" if is_lut_active == 1 else "LUT:RAW"

        if is_collided == 1:
            ax3.set_facecolor('#ffe6e6')
            fig.suptitle(
                f"[CANH BAO VA CHAM GMO!] DUNG KHAN CAP -> TAI KHOI DONG SAU: {countdown_sec}s | Tau_ext: {latest_tau_ext:+.3f} N.m",
                fontsize=11, fontweight="bold", color="red"
            )
        else:
            ax3.set_facecolor('white')
            cmd_info = f"Iq* = {current_target_iq:+.2f}A" if is_torque_mode == 1 else f"RPM* = {current_target_rpm:+5.0f}"
            fig.suptitle(
                f"[{mode_str}: {cmd_info}]  |  RPM: {latest_rpm:+5.1f} | Iq: {latest_iq:+.2f}A | Uq: {latest_vq:+.1f}V | Tau: {latest_tau_ext:+.3f}/{latest_tau_thresh:.3f}Nm | [{lut_str}] [{fw_str}]",
                fontsize=10, fontweight="bold", color="darkblue"
            )

        return line_rpm, line_rpm_ref, line_iq, line_iq_ref, line_id, line_tau, line_th_up, line_th_dn

    ani = animation.FuncAnimation(fig, animate, interval=30, blit=False, cache_frame_data=False)
    plt.tight_layout()
    plt.show()


def run_menu():
    print("=" * 78)
    print("   20kHz MIN-MAX FOC + GENERALIZED MOMENTUM OBSERVER (GMO) - GIM8108")
    print("   [t] Chuyen Mode: SPEED (RPM) <---> TORQUE (Iq) | [a] Arm DRV8301")
    print("   [+] / [-] : Tang / Giam Muc Tieu (+/- 10 RPM hoac +/- 0.05 A)")
    print("   [Space]   : Dung khan cap")
    print("   [i] Mo 3 Bang Scope | [f] FW | [m] LUT | [c] Calib | [q] Thoat")
    print("=" * 78)

    last_print = 0
    while True:
        if time.time() - last_print > 0.1:
            last_print = time.time()
            st_text  = STATE_SHORT.get(latest_state, "UNK")
            mode_str = "TRQ" if is_torque_mode == 1 else "SPD"

            if is_collided == 1:
                line_str = f">> [VA CHAM GMO!] DUNG MO-TO -> CHAY LAI TRONG: {countdown_sec}s | Tau: {latest_tau_ext:+.3f}Nm"
            else:
                if is_torque_mode == 1:
                    line_str = f">> [{mode_str}] Iq*:{current_target_iq:+.2f}A | Iq:{latest_iq:+.2f}A | RPM:{latest_rpm:+5.1f} | Tau:{latest_tau_ext:+.3f}Nm | [{st_text}]"
                else:
                    line_str = f">> [{mode_str}] RPM*:{current_target_rpm:+4.0f} | RPM:{latest_rpm:+5.1f} | Iq:{latest_iq:+.2f}A | Tau:{latest_tau_ext:+.3f}Nm | [{st_text}]"

            sys.stdout.write(f"\r{line_str.ljust(78)}")
            sys.stdout.flush()

        if msvcrt.kbhit():
            ch = msvcrt.getch().decode('ascii', errors='ignore')
            if ch in ['i', 'I']:
                show_foc_gmo_scope()
            elif ch in ['q', 'Q']:
                send_can_cmd(0x01)
                print("\nDa thoat.")
                break
            else:
                process_key(ch)

        time.sleep(0.01)


if __name__ == "__main__":
    try:
        run_menu()
    except KeyboardInterrupt:
        send_can_cmd(0x01)
        ser.close()