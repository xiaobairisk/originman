#!/usr/bin/python3
# coding=utf8
import sys
import os
import time
import pygame
import threading
import hiwonder.ActionGroupControl as AGC

#ps2手柄控制动作, 已以system方式自启动，无需再开启(PS2 controller controls the actions. It's already set to auto-start using the system method, so there's no need to start it again)

key_map = {"PSB_CROSS":0, "PSB_CIRCLE":1, "PSB_SQUARE":3, "PSB_TRIANGLE":4,
        "PSB_L1": 6, "PSB_R1":7, "PSB_L2":8, "PSB_R2":9,
        "PSB_SELECT":10, "PSB_START":11, "PSB_L3":13, "PSB_R3":14}
# key_map = {"PSB_CROSS":2, "PSB_CIRCLE":1, "PSB_SQUARE":3, "PSB_TRIANGLE":0,
#         "PSB_L1": 4, "PSB_R1":5, "PSB_L2":6, "PSB_R2":7,
#         "PSB_SELECT":8, "PSB_START":9, "PSB_L3":10, "PSB_R3":11};
action_map = ["CROSS", "CIRCLE", "", "SQUARE", "TRIANGLE", "L1", "R1", "L2", "R2", "SELECT", "START", "", "L3", "R3"]

def joystick_init():
    os.environ["SDL_VIDEODRIVER"] = "dummy"
    pygame.display.init()
    pygame.joystick.init()
    if pygame.joystick.get_count() > 0:
        js=pygame.joystick.Joystick(0)
        js.init()
        jsName = js.get_name()
        print("Name of the joystick:", jsName)
        jsAxes=js.get_numaxes()
        print("Number of axif:",jsAxes)
        jsButtons=js.get_numbuttons()
        print("Number of buttons:", jsButtons);
        jsBall=js.get_numballs()
        print("Numbe of balls:", jsBall)
        jsHat= js.get_numhats()
        print("Number of hats:", jsHat)

th = None
last_status = ''
connected = False
while True:
    if os.path.exists("/dev/input/js0") is True:
        if connected is False:
            joystick_init()
            jscount =  pygame.joystick.get_count()
            if jscount > 0:
                try:
                    js=pygame.joystick.Joystick(0)
                    js.init()
                    connected = True
                except Exception as e:
                    print(e)
            else:
                pygame.joystick.quit()
    else:
        if connected is True:
            connected = False
            js.quit()
            pygame.joystick.quit()
    if connected is True:
        pygame.event.pump()     
        actName = None
        times = 1
        try:
            if js.get_button(key_map["PSB_R1"]):
                actName = 'right_kick'
            if js.get_button(key_map["PSB_R2"]):
                actName = 'turn_right'
            if js.get_button(key_map["PSB_L1"]):
                actName = 'left_kick'
            if js.get_button(key_map["PSB_L2"]):
                actName = 'turn_left'
            if js.get_button(key_map["PSB_SQUARE"]): #正方形(square)
                actName = 'left_shot_fast'
            if js.get_button(key_map["PSB_CIRCLE"]): #圈(circle)
                actName = 'right_shot_fast'
            if js.get_button(key_map["PSB_TRIANGLE"]): #三角(triangle)
                actName = 'wave'
            if js.get_button(key_map["PSB_CROSS"]): #叉(cross)
                actName = 'bow'
                
            lx = js.get_axis(0)
            ly = js.get_axis(1)
            if lx < -0.5 :
                actName = 'left_move_fast'
            elif lx > 0.5:             
                actName = 'right_move_fast'
            l3_state = js.get_button(key_map["PSB_L3"])
            if ly < -0.5 :
                if not l3_state:
                    last_status = 'go'
                    actName = 'go_forward'
                    times = 0
            elif ly > 0.5:
                if not l3_state:
                    last_status = 'back'
                    actName = 'back_fast'
                    times = 0
            else:
                if (last_status == 'go' or last_status == 'back') and actName is None:
                    AGC.stopActionGroup()
                    last_status = ''
            if js.get_button(key_map["PSB_START"]):
                actName = 'stand_slow'
            if th is not None:
                if actName is not None:
                    if not th.is_alive():
                        th = threading.Thread(target=AGC.runActionGroup, args=(actName, times), daemon=True)
                        th.start()
            else:
                th = threading.Thread(target=AGC.runActionGroup, args=(actName, times), daemon=True)
                th.start()
            # print(actName)
        except Exception as e:
            print(e)
            connected = False          
    time.sleep(0.01)
