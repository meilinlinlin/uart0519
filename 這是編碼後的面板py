import tkinter as tk
import threading
import queue
import serial
import time

# ====== 這裡改成你的 STM32 COM Port ======
COM_PORT = "COM9"
BAUD_RATE = 115200

event_queue = queue.Queue()

current_blink_job = None
blink_visible = True
ser = None


# =========================================================
# UART / Serial
# =========================================================
def serial_init():
    global ser

    try:
        ser = serial.Serial(COM_PORT, BAUD_RATE, timeout=1)
        time.sleep(2)
        print(f"[Serial] 已連線到 {COM_PORT}, baud = {BAUD_RATE}")
    except Exception as e:
        ser = None
        print("[Serial] 連線失敗，請確認 COM Port")
        print(e)


def send_to_stm32(code):
    global ser

    if ser is None:
        print("[Serial] 尚未連線，無法送出")
        return

    try:
        ser.write(code.encode("utf-8"))
        ser.flush()
        print(f"[Serial] 已送出：{code}")
    except Exception as e:
        print("[Serial] 傳送失敗")
        print(e)


def serial_read_thread():
    global ser

    while True:
        try:
            if ser is not None and ser.is_open:
                line = ser.readline().decode("utf-8", errors="ignore").strip()
                if line:
                    print(f"[STM32] {line}")
            else:
                time.sleep(0.5)

        except Exception as e:
            print("[Serial] 讀取失敗")
            print(e)
            time.sleep(1)


# =========================================================
# Blink control
# =========================================================
def stop_blink():
    global current_blink_job, blink_visible

    if current_blink_job is not None:
        root.after_cancel(current_blink_job)
        current_blink_job = None

    blink_visible = True


def blink_panel(color, interval_ms):
    global blink_visible, current_blink_job

    blink_visible = not blink_visible

    if blink_visible:
        frame.config(highlightbackground=color)
        label_warning.config(fg=color)
        label_icons.config(fg=color)
    else:
        frame.config(highlightbackground="black")
        label_warning.config(fg="black")
        label_icons.config(fg="black")

    current_blink_job = root.after(interval_ms, lambda: blink_panel(color, interval_ms))


# =========================================================
# Panel display functions
# =========================================================
def show_safe():
    stop_blink()

    frame.config(
        highlightbackground="#00FF66",
        highlightthickness=6
    )

    label_warning.config(
        text="",
        fg="#00FF66"
    )

    label_icons.config(
        text="",
        fg="#00FF66"
    )

    label_title.config(
        text="安全",
        fg="white"
    )

    label_msg.config(
        text="目前未偵測到弱勢用路人",
        fg="white"
    )

    label_code.config(
        text="事件代碼：0｜Payload：00000000",
        fg="#00FF66"
    )


def show_vulnerable_road_user(level):
    stop_blink()

    if level == 1:
        color = "#FFD700"
        thickness = 8
        warning_text = "⚠️"
        title = "弱勢用路人穿越｜Level 1 注意"
        msg = "請注意前方路況，建議提前減速"
        code_text = "事件代碼：1｜Payload：00101000"
        blink_ms = 750

    elif level == 2:
        color = "#FF8800"
        thickness = 10
        warning_text = "⚠️"
        title = "弱勢用路人穿越｜Level 2 警告"
        msg = "請減速，前方弱勢用路人風險提高"
        code_text = "事件代碼：2｜Payload：00110011"
        blink_ms = 500

    elif level == 3:
        color = "#FF3333"
        thickness = 14
        warning_text = "🚨"
        title = "弱勢用路人穿越｜Level 3 緊急"
        msg = "請立即減速或煞車，前方高風險"
        code_text = "事件代碼：3｜Payload：00111010"
        blink_ms = 300

    else:
        show_unknown()
        return

    frame.config(
        highlightbackground=color,
        highlightthickness=thickness
    )

    label_warning.config(
        text=warning_text,
        fg=color
    )

    label_icons.config(
        text="🚶   🚲   🛵",
        fg=color
    )

    label_title.config(
        text=title,
        fg="white"
    )

    label_msg.config(
        text=msg,
        fg="white"
    )

    label_code.config(
        text=code_text,
        fg=color
    )

    blink_panel(color, blink_ms)


def show_system_error():
    stop_blink()

    color = "#B066FF"

    frame.config(
        highlightbackground=color,
        highlightthickness=12
    )

    label_warning.config(
        text="ERROR",
        fg=color
    )

    label_icons.config(
        text="系統異常",
        fg=color
    )

    label_title.config(
        text="系統錯誤",
        fg="white"
    )

    label_msg.config(
        text="感測器、辨識端或通訊狀態異常，請檢查系統",
        fg="white"
    )

    label_code.config(
        text="事件代碼：E｜Payload：11100111",
        fg=color
    )

    blink_panel(color, 450)


def show_unknown():
    stop_blink()

    frame.config(
        highlightbackground="white",
        highlightthickness=6
    )

    label_warning.config(
        text="--",
        fg="white"
    )

    label_icons.config(
        text="",
        fg="white"
    )

    label_title.config(
        text="未知輸入",
        fg="white"
    )

    label_msg.config(
        text="請輸入 0、1、2、3 或 E",
        fg="white"
    )

    label_code.config(
        text="事件代碼：無效",
        fg="red"
    )


# =========================================================
# Event command mapping
# =========================================================
def show_event(code):
    code = code.strip().upper()

    if code == "0":
        show_safe()
        send_to_stm32("0")

    elif code == "1":
        show_vulnerable_road_user(1)
        send_to_stm32("1")

    elif code == "2":
        show_vulnerable_road_user(2)
        send_to_stm32("2")

    elif code == "3":
        show_vulnerable_road_user(3)
        send_to_stm32("3")

    elif code == "E":
        show_system_error()
        send_to_stm32("E")

    else:
        show_unknown()


def terminal_input_thread():
    print("請在終端機輸入事件代碼：")
    print("0 = 無事件 / 安全")
    print("1 = 弱勢用路人穿越 Level 1 注意")
    print("2 = 弱勢用路人穿越 Level 2 警告")
    print("3 = 弱勢用路人穿越 Level 3 緊急")
    print("E = 系統錯誤")
    print("-" * 50)

    while True:
        code = input("請輸入 0 / 1 / 2 / 3 / E：")
        event_queue.put(code)


def check_queue():
    while not event_queue.empty():
        code = event_queue.get()
        show_event(code)

    root.after(100, check_queue)


def on_close():
    global ser

    try:
        if ser is not None and ser.is_open:
            ser.close()
            print("[Serial] 已關閉")
    except:
        pass

    root.destroy()


# =========================================================
# Main program
# =========================================================
serial_init()

root = tk.Tk()
root.title("公車後方警示面板模擬")
root.geometry("980x720")
root.configure(bg="black")
root.protocol("WM_DELETE_WINDOW", on_close)

frame = tk.Frame(
    root,
    bg="black",
    highlightbackground="#00FF66",
    highlightthickness=6
)
frame.pack(expand=True, fill="both", padx=30, pady=30)

label_warning = tk.Label(
    frame,
    text="",
    font=("Arial", 95, "bold"),
    fg="white",
    bg="black"
)
label_warning.pack(pady=(45, 5))

label_icons = tk.Label(
    frame,
    text="",
    font=("Arial", 60, "bold"),
    fg="white",
    bg="black"
)
label_icons.pack(pady=(10, 15))

label_title = tk.Label(
    frame,
    text="安全",
    font=("Microsoft JhengHei", 44, "bold"),
    fg="white",
    bg="black"
)
label_title.pack(pady=(15, 5))

label_msg = tk.Label(
    frame,
    text="目前未偵測到弱勢用路人",
    font=("Microsoft JhengHei", 25),
    fg="white",
    bg="black"
)
label_msg.pack(pady=(10, 8))

label_code = tk.Label(
    frame,
    text="事件代碼：0｜Payload：00000000",
    font=("Microsoft JhengHei", 21, "bold"),
    fg="#00FF66",
    bg="black"
)
label_code.pack(pady=(10, 20))

# 啟動時先顯示安全狀態
show_safe()

thread = threading.Thread(target=terminal_input_thread, daemon=True)
thread.start()

serial_thread = threading.Thread(target=serial_read_thread, daemon=True)
serial_thread.start()

root.after(100, check_queue)
root.mainloop()
