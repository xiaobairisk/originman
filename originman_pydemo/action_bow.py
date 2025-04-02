#!/usr/bin/env python3
# encoding:utf-8
import time
import hiwonder.ActionGroupControl

class RobotActionController:
    def __init__(self):
        pass

    def run_action_group(self, action_group_name, wait_time=5):
        """
        运行指定的动作组
        :param action_group_name: 动作组文件名（不包含后缀.dfca）
        :param wait_time: 执行完动作组后的等待时间（秒），默认5秒
        """
        try:
            hiwonder.ActionGroupControl.runActionGroup(action_group_name)
            # 根据动作组的预期执行时间等待
            print(f"执行动作组: {action_group_name}")
            time.sleep(wait_time)  # 等待动作组完成
        except Exception as e:
            print(f"执行动作组 {action_group_name} 时出错: {e}")

if __name__ == "__main__":
    controller = RobotActionController()
    
    # 先执行 'stand' 动作组
    controller.run_action_group("stand")
    
    # 然后执行 'bow' 动作组
    controller.run_action_group("bow")
