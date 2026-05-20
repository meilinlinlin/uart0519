import serial
import threading
import time

PORT = "COM9"        # 改成你的 USB-TTL COM Port，例如 COM6、COM9
BAUDRATE = 115200

def receive_data(ser):
    while True:
        try:
            data = ser.readline().decode("utf-8", errors="ignore").strip()
            if data:
                print(f"\nSTM32 > {data}")
        except:
            print("\n接收中斷")
            break

def main():
    try:
        ser = serial.Serial(PORT, BAUDRATE, timeout=1)
        time.sleep(2)

        print(f"已開啟 {PORT}")
        print("輸入文字後按 Enter 傳給 STM32")
        print("例如輸入 abc")
        print("輸入 exit 離開\n")

        rx_thread = threading.Thread(target=receive_data, args=(ser,), daemon=True)
        rx_thread.start()

        while True:
            msg = input("PC > ")

            if msg.lower() == "exit":
                break

            ser.write((msg + "\r\n").encode("utf-8"))

        ser.close()
        print("Serial 已關閉")

    except serial.SerialException as e:
        print("Serial 開啟失敗：", e)
        print("請確認 COM Port 是否正確，還有 Tera Term 有沒有關掉。")

if __name__ == "__main__":
    main()
