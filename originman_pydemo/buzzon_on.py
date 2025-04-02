import time
from ros_robot_controller_sdk import Board

def control_buzzer(board, frequency, on_time, off_time, repeat):
    """
    控制蜂鸣器按照指定参数发声。
    :param board: 已初始化的Board对象
    :param frequency: 蜂鸣器发声频率
    :param on_time: 发声持续时间
    :param off_time: 停止持续时间
    :param repeat: 重复次数
    """
    if board is None:
        print("Board is not initialized. Cannot control the buzzer.")
        return
    try:
        board.set_buzzer(frequency, on_time, off_time, repeat)
        total_time = (on_time + off_time) * repeat
        time.sleep(total_time)
    except Exception as e:
        print(f"Error controlling the buzzer: {e}")

def main():
    # 初始化Board
    board = Board(device="/dev/ttyS1", baudrate=1000000)
    board.enable_reception()
    print("Board initialized")
    
    # 第一次控制蜂鸣器发声
    control_buzzer(board, 1900, 0.1, 0.9, 2)
    # 第二次控制蜂鸣器发声
    control_buzzer(board, 1000, 0.5, 0.5, 1)


if __name__ == "__main__":
    main()

