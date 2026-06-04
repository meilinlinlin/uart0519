import serial
import time
import re


# =========================
# 使用者設定區
# =========================

PORT = "COM10"              # STM32 UART4 的 COM Port
BAUD_RATE = 115200          # STM32 UART4 的鮑率

ROUND_COUNT = 3             # 每次最終判斷讀取 3 組結果
THRESHOLD_COUNT = 35        # 每組 100 筆資料中，至少 35 筆為 1 就判定該組為 1

RUN_CONTINUOUSLY = True     # True：持續判斷；False：完成一組三次判斷後停止


# =========================
# 解析 STM32 傳來的 count
# =========================

def parse_count_result(line):
    """
    支援以下 STM32 輸出格式：

    IR received, count = 42 / 100
    No IR, count = 23 / 100
    IR received, count = 42 / 100 | Result: 1
    No IR, count = 23 / 100 | Result: 0

    注意：
    Python 不採用 STM32 訊息中的 IR received / No IR 作為判定依據，
    而是直接用 count 與 THRESHOLD_COUNT 重新判斷。
    """

    line = line.strip()

    match = re.search(r"count\s*=\s*(\d+)\s*/\s*(\d+)", line)

    if match is None:
        return None

    hit_count = int(match.group(1))
    sample_count = int(match.group(2))

    # 由 Python 根據目前門檻重新判斷單組結果
    result = 1 if hit_count >= THRESHOLD_COUNT else 0

    return {
        "Raw_Data": line,
        "Hit_Count": hit_count,
        "Sample_Count": sample_count,
        "Result": result
    }


# =========================
# 主程式
# =========================

def main():
    print("=" * 65)
    print("IR 三組判斷測試開始")
    print(f"COM Port       : {PORT}")
    print(f"Baud Rate      : {BAUD_RATE}")
    print(f"單組判斷門檻   : count >= {THRESHOLD_COUNT} / 100 時，該組判定為 1")
    print(f"最終判斷方式   : {ROUND_COUNT} 組中只要有一組為 1，最終輸出 1")
    print("=" * 65)

    group_number = 0

    try:
        with serial.Serial(PORT, BAUD_RATE, timeout=2) as ser:
            time.sleep(2)

            while True:
                group_number += 1
                round_results = []

                print(f"\n第 {group_number} 組三次判斷：")

                while len(round_results) < ROUND_COUNT:
                    raw = ser.readline().decode("utf-8", errors="ignore").strip()

                    if raw == "":
                        continue

                    parsed = parse_count_result(raw)

                    # STM32 UART Ready、IR RX mode 等訊息直接顯示但不列入三次
                    if parsed is None:
                        print(f"系統訊息：{raw}")
                        continue

                    round_results.append(parsed["Result"])

                    print(
                        f"收到資料：count = {parsed['Hit_Count']:>3} / {parsed['Sample_Count']:<3} "
                        f"| 判定：{parsed['Result']} |"
                    )

                # 三組中只要有一組為 1，最終即判定為 1
                success_rounds = sum(round_results)
                final_result = 1 if success_rounds >= 1 else 0

                print(f"三次判斷：{success_rounds}/{ROUND_COUNT} 輸出{final_result}")
                print("-" * 65)

                if not RUN_CONTINUOUSLY:
                    break

    except serial.SerialException as error:
        print("無法開啟 COM Port。")
        print(f"錯誤內容：{error}")
        print("請確認：")
        print("1. COM Port 是否正確，例如 COM10")
        print("2. STM32 是否已連接電腦")
        print("3. Tera Term 或 Serial Monitor 是否正在占用 COM Port")

    except KeyboardInterrupt:
        print("\n程式已停止。")


if __name__ == "__main__":
    main()