import time
import serial
from serial.tools import list_ports


BAUD_RATE = 115200

# PC 端只傳送事件指令字元；
# 6-bit Payload 與 CRC-2 由 STM32 發送端自行產生。
VALID_COMMANDS = {
    "0": "SAFE",
    "1": "VRU Level 1",
    "2": "VRU Level 2",
    "3": "VRU Level 3",
    "4": "BRAKE Level 1",
    "5": "BRAKE Level 2",
    "6": "BRAKE Level 3",
    "E": "SYSTEM ERROR",
}


def show_ports() -> None:
    ports = list(list_ports.comports())

    if not ports:
        print("目前找不到 COM Port")
        return

    print("可用的 COM Port：")
    for port in ports:
        print(f"  {port.device}: {port.description}")


def show_commands() -> None:
    print("\n可用指令：")
    for command, description in VALID_COMMANDS.items():
        print(f"  {command} = {description}")

    print("  q = 離開")


def main() -> None:
    show_ports()

    port_name = input(
        "\n請輸入 USB-UART 的 COM Port，例如 COM5："
    ).strip()

    try:
        with serial.Serial(
            port=port_name,
            baudrate=BAUD_RATE,
            timeout=0.1,
            write_timeout=1,
        ) as ser:

            # 等待 STM32 與 USB-UART 穩定
            time.sleep(2)

            # 清除程式剛啟動時殘留的資料
            ser.reset_input_buffer()
            ser.reset_output_buffer()

            print(
                f"\n已連線到 {port_name}, "
                f"baud = {BAUD_RATE}"
            )

            show_commands()

            while True:
                command = input("\nPC send: ").strip()

                if command.lower() == "q":
                    break

                # 小寫 e 統一轉成大寫 E
                if command.lower() == "e":
                    command = "E"

                if command not in VALID_COMMANDS:
                    print(
                        "只能輸入 0、1、2、3、4、5、6、E 或 q"
                    )
                    continue

                # 清除上一個事件可能留下的 STM32 回覆
                ser.reset_input_buffer()

                # 傳送單一 ASCII 指令給 STM32
                ser.write(command.encode("ascii"))
                ser.flush()

                print(
                    f"已送出：{command} = "
                    f"{VALID_COMMANDS[command]}"
                )

                print("STM32 reply:")

                # 最多等 2 秒，直到收到 TIME 開頭的量測結果
                deadline = time.monotonic() + 2.0
                received_time_result = False

                while time.monotonic() < deadline:
                    raw_line = ser.readline()

                    if not raw_line:
                        continue

                    line = raw_line.decode(
                        "utf-8",
                        errors="ignore"
                    ).strip()

                    if not line:
                        continue

                    print(line)

                    # STM32 完成 IR 與 LCD 量測後，
                    # 會傳回 TIME 開頭的資料
                    if line.startswith("TIME,"):
                        received_time_result = True
                        break

                if not received_time_result:
                    print(
                        "2 秒內沒有收到 TIME 回覆，"
                        "請檢查 STM32 程式、UART 接線或鮑率"
                    )

    except serial.SerialException as error:
        print(f"\n串口連線失敗：{error}")
        print(
            "請確認 COM Port、接線，以及 COM Port "
            "沒有被其他程式占用。"
        )


if __name__ == "__main__":
    main()
