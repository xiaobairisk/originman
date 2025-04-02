#!/usr/bin/env python3
# encoding: utf-8
import os
import re
import sys
import copy
import time
import threading
import sqlite3
import resource_rc
import hiwonder.ActionGroupControl as AGC
import hiwonder.ros_robot_controller_sdk as rrc

from hiwonder.Controller import Controller
from Ui import Ui_Form
from PyQt5.QtGui import *
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5 import QtGui, QtWidgets
from PyQt5.QtSql import QSqlDatabase, QSqlQuery

def run_action_group(num):
    threading.Thread(target=AGC.runAction, args=(num, )).start()    

def stop_action_group():    
    AGC.stopAction() 

class MainWindow(QtWidgets.QWidget, Ui_Form):
    def __init__(self):
        super(MainWindow, self).__init__()        
        self.setupUi(self)    

        if self.HandEnabled.isChecked():
            self.joints = 18
            self.widget_id1_17.show()
            self.widget_id1_18.show()
        else:
            self.joints = 16
            self.widget_id1_17.hide()
            self.widget_id1_18.hide()

        #self.joints = 16
        self.all_joints = 18
        self.min_time = 20
        self.chinese = True
        self.path = '/userdata/dev_ws/src/originman/driver/'
        self.actdir = self.path + "ActionGroups/"        
        self.tableWidget.setSelectionBehavior(QAbstractItemView.SelectRows)  # 设置选中整行，若不设置默认选中单元格(set to select the entire row. If not set, by default, individual cells will be selected)
        self.message = QMessageBox()
        # os.system("sudo systemctl stop originman")
        self.board = rrc.Board()
        self.board.enable_reception()
        self.ctl = Controller(self.board)
        self.running = False
        self.horizontalSlider_manual = True
        self.horizontalSlider_dev_manual = True 
         
        self.reflash_action()
        ########################主界面(main interface)###############################
        self.radioButton_zn.toggled.connect(lambda: self.language(self.radioButton_zn))
        self.radioButton_en.toggled.connect(lambda: self.language(self.radioButton_en))
        self.HandEnabled.toggled.connect(self.hand_state)
        # 根据系统语言进行切换(switch based on the system language)
        result1 = os.popen('locale |grep LC_ALL').read().replace('\n', '').split('=')[1]
        result2 = os.popen('locale |grep LANG').read().replace('\n', '').split('=')[1]
        
        if result1 == '':
            if 'CN' in result2:
                self.radioButton_zn.setChecked(True)
            else:
                self.radioButton_en.setChecked(True)
        else:
            if 'CN' in result1:
                self.radioButton_zn.setChecked(True)
            else:
                self.radioButton_en.setChecked(True)        
        
        self.lineEdit_object = []    
        for i in range(self.all_joints):
            self.lineEdit_object.append(self.findChild(QLineEdit, "lineEdit_id" + str(i + 1)))            
        self.validator = QIntValidator(0, 1000)
        for i in range(self.all_joints):
            self.lineEdit_object[i].setValidator(self.validator)
            self.lineEdit_object[i].installEventFilter(self)

        self.horizontalSlider_pulse_object = []
        for i in range(self.all_joints):
            self.horizontalSlider_pulse_object.append(self.findChild(QSlider, "horizontalSlider_pulse_id" + str(i + 1)))
        # 滑竿同步对应文本框的数值,及滑竿控制相应舵机转动与valuechange函数绑定(synchronize the slider with the corresponding text box value, and bind the slider control to the corresponding servo rotation with the 'valueChange' function)
        self.horizontalSlider_pulse_object[0].valueChanged.connect(lambda: self.pulse_valuechange(0))
        self.horizontalSlider_pulse_object[1].valueChanged.connect(lambda: self.pulse_valuechange(1))
        self.horizontalSlider_pulse_object[2].valueChanged.connect(lambda: self.pulse_valuechange(2))
        self.horizontalSlider_pulse_object[3].valueChanged.connect(lambda: self.pulse_valuechange(3))
        self.horizontalSlider_pulse_object[4].valueChanged.connect(lambda: self.pulse_valuechange(4))
        self.horizontalSlider_pulse_object[5].valueChanged.connect(lambda: self.pulse_valuechange(5))
        self.horizontalSlider_pulse_object[6].valueChanged.connect(lambda: self.pulse_valuechange(6))
        self.horizontalSlider_pulse_object[7].valueChanged.connect(lambda: self.pulse_valuechange(7))
        self.horizontalSlider_pulse_object[8].valueChanged.connect(lambda: self.pulse_valuechange(8))
        self.horizontalSlider_pulse_object[9].valueChanged.connect(lambda: self.pulse_valuechange(9))
        self.horizontalSlider_pulse_object[10].valueChanged.connect(lambda: self.pulse_valuechange(10))
        self.horizontalSlider_pulse_object[11].valueChanged.connect(lambda: self.pulse_valuechange(11))
        self.horizontalSlider_pulse_object[12].valueChanged.connect(lambda: self.pulse_valuechange(12))
        self.horizontalSlider_pulse_object[13].valueChanged.connect(lambda: self.pulse_valuechange(13))
        self.horizontalSlider_pulse_object[14].valueChanged.connect(lambda: self.pulse_valuechange(14))
        self.horizontalSlider_pulse_object[15].valueChanged.connect(lambda: self.pulse_valuechange(15))
        self.horizontalSlider_pulse_object[16].valueChanged.connect(lambda: self.pulse_valuechange(16))
        self.horizontalSlider_pulse_object[17].valueChanged.connect(lambda: self.pulse_valuechange(17))

        self.horizontalSlider_dev_object = []
        for i in range(self.all_joints):
            self.horizontalSlider_dev_object.append(self.findChild(QSlider, "horizontalSlider_dev_id" + str(i + 1)))        
        self.horizontalSlider_dev_object[0].valueChanged.connect(lambda: self.dev_valuechange(0))
        self.horizontalSlider_dev_object[1].valueChanged.connect(lambda: self.dev_valuechange(1))
        self.horizontalSlider_dev_object[2].valueChanged.connect(lambda: self.dev_valuechange(2))
        self.horizontalSlider_dev_object[3].valueChanged.connect(lambda: self.dev_valuechange(3))
        self.horizontalSlider_dev_object[4].valueChanged.connect(lambda: self.dev_valuechange(4))
        self.horizontalSlider_dev_object[5].valueChanged.connect(lambda: self.dev_valuechange(5))
        self.horizontalSlider_dev_object[6].valueChanged.connect(lambda: self.dev_valuechange(6))
        self.horizontalSlider_dev_object[7].valueChanged.connect(lambda: self.dev_valuechange(7))
        self.horizontalSlider_dev_object[8].valueChanged.connect(lambda: self.dev_valuechange(8))
        self.horizontalSlider_dev_object[9].valueChanged.connect(lambda: self.dev_valuechange(9))
        self.horizontalSlider_dev_object[10].valueChanged.connect(lambda: self.dev_valuechange(10))
        self.horizontalSlider_dev_object[11].valueChanged.connect(lambda: self.dev_valuechange(11))
        self.horizontalSlider_dev_object[12].valueChanged.connect(lambda: self.dev_valuechange(12))
        self.horizontalSlider_dev_object[13].valueChanged.connect(lambda: self.dev_valuechange(13))
        self.horizontalSlider_dev_object[14].valueChanged.connect(lambda: self.dev_valuechange(14))
        self.horizontalSlider_dev_object[15].valueChanged.connect(lambda: self.dev_valuechange(15))
        self.horizontalSlider_dev_object[16].valueChanged.connect(lambda: self.dev_valuechange(16))
        self.horizontalSlider_dev_object[17].valueChanged.connect(lambda: self.dev_valuechange(17))
        self.label_object = []
        for i in range(self.all_joints):
            self.label_object.append(self.findChild(QLabel, "label_id" + str(i + 1))) 

        # tableWidget点击获取定位的信号与icon_position函数（添加运行图标）绑定(bind the signal for clicking the tableWidget to locate with the 'icon_position' function (to add the running icon))
        self.tableWidget.pressed.connect(self.icon_position)

        self.validator3 = QIntValidator(20, 30000)
        self.lineEdit_time.setValidator(self.validator3)

        # 将编辑动作组的按钮点击时的信号与button_editaction_clicked函数绑定(bind the signal for clicking the button to edit action groups with the 'button_editaction_clicked' function)
        self.Button_ServoPowerDown.pressed.connect(lambda: self.button_editaction_clicked('servoPowerDown'))
        self.Button_AngularReadback.pressed.connect(lambda: self.button_editaction_clicked('angularReadback'))
        self.Button_AddAction.pressed.connect(lambda: self.button_editaction_clicked('addAction'))
        self.Button_DelectAction.pressed.connect(lambda: self.button_editaction_clicked('delectAction'))
        
        self.Button_UpdateAction.pressed.connect(lambda: self.button_editaction_clicked('updateAction'))
        self.Button_InsertAction.pressed.connect(lambda: self.button_editaction_clicked('insertAction'))
        self.Button_MoveUpAction.pressed.connect(lambda: self.button_editaction_clicked('moveUpAction'))
        self.Button_MoveDownAction.pressed.connect(lambda: self.button_editaction_clicked('moveDownAction'))        

        # 将运行及停止运行按钮点击的信号与button_runonline函数绑定(bind the signal for clicking the buttons to run and stop with the 'button_runonline' function)
        self.Button_Run.clicked.connect(lambda: self.button_run('run'))
        
        self.icon = QtGui.QIcon()
        self.icon.addPixmap(QtGui.QPixmap(":/images/index.png"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        
        self.Button_OpenActionGroup.pressed.connect(lambda: self.button_flie_operate('openActionGroup'))
        self.Button_SaveActionGroup.pressed.connect(lambda: self.button_flie_operate('saveActionGroup'))
        self.Button_ReadDeviation.pressed.connect(lambda: self.button_flie_operate('readDeviation'))
        self.Button_DownloadDeviation.pressed.connect(lambda: self.button_flie_operate('downloadDeviation'))
        self.Button_TandemActionGroup.pressed.connect(lambda: self.button_flie_operate('tandemActionGroup'))
        self.Button_ReSetServos.pressed.connect(lambda: self.button_re_clicked('reSetServos'))
        
        # 将控制动作的按钮点击的信号与action_control_clicked函数绑定(bind the signal for clicking the buttons to control actions with the 'action_control_clicked' function)
        self.Button_DelectSingle.pressed.connect(lambda: self.button_controlaction_clicked('delectSingle'))
        self.Button_AllDelect.pressed.connect(lambda: self.button_controlaction_clicked('allDelect'))
        self.Button_RunAction.pressed.connect(lambda: self.button_controlaction_clicked('runAction'))
        self.Button_StopAction.pressed.connect(lambda: self.button_controlaction_clicked('stopAction'))                
        
        self.tableWidget.setContextMenuPolicy(Qt.CustomContextMenu)  ######允许右键产生子菜单(allow right click to generate sub-menus)
        self.tableWidget.customContextMenuRequested.connect(self.generate_menu)  ####右键菜单(right click menu)

        self.devNew = self.all_joints*[0]
        self.dev_change = False 
        self.readDevOk = False
        self.totalTime = 0
        self.row = 0
        
        self.use_time_list = []
        self.loop = False
        
    def hand_state(self):
        if self.HandEnabled.isChecked():
            self.joints = 18
            self.widget_id1_17.show()
            self.widget_id1_18.show()

        else:
            self.joints = 16
            self.widget_id1_17.hide()
            self.widget_id1_18.hide()    

    def generate_menu(self, pos):
        menu = QMenu()
        if self.chinese:
            item1 = menu.addAction("删除全部动作")
            item2 = menu.addAction("镜像动作")
            item3 = menu.addAction("镜像全部动作")
        else:
            item1 = menu.addAction("Delect all action")
            item2 = menu.addAction("Mirror action")
            item3 = menu.addAction("Mirror all action")
        action = menu.exec_(self.tableWidget.mapToGlobal(pos))
        if action == item1:
            self.button_editaction_clicked("delectAllAction")
        elif action == item2:
            self.button_editaction_clicked("mirrorAction")
        elif action == item3:
            self.button_editaction_clicked("mirrorAllAction")
        else:
            return

    # 弹窗提示函数(pop-up window prompt function)
    def message_from(self, str):
        try:
            QMessageBox.about(self, '', str)
        except:
            pass
   
    # 弹窗提示函数(pop-up window prompt function)
    def message_delect(self, str):
        messageBox = QMessageBox()
        messageBox.setWindowTitle(' ')
        messageBox.setText(str)
        messageBox.addButton(QPushButton('OK'), QMessageBox.YesRole)
        messageBox.addButton(QPushButton('Cancel'), QMessageBox.NoRole)
        return messageBox.exec_()

    # 窗口退出(window exit)
    def closeEvent(self, e):        
        result = QMessageBox.question(self,
                                    "main",
                                    "Quit?",
                                    QMessageBox.Yes | QMessageBox.No,
                                    QMessageBox.No)
        if result == QMessageBox.Yes:
            # os.system("sudo systemctl restart tonypi")
            QWidget.closeEvent(self, e)           
        else:
            e.ignore()

    def language(self, name):
        QToolTip.setFont(QFont("SansSerif", 10))
        if name.text() == "中文":
            self.chinese = True
            self.Button_ServoPowerDown.setText("手掰编程")
            self.Button_AngularReadback.setText("角度回读")
            self.Button_AddAction.setText("添加动作")
            self.Button_DelectAction.setText("删除动作")
            self.Button_UpdateAction.setText("更新动作")
            self.Button_InsertAction.setText("插入动作")
            self.Button_MoveUpAction.setText("上移动作")
            self.Button_MoveDownAction.setText("下移动作")        
            self.Button_OpenActionGroup.setText("打开动作文件")
            self.Button_SaveActionGroup.setText("保存动作文件")
            self.Button_ReadDeviation.setText("读取偏差")
            self.Button_DownloadDeviation.setText("下载偏差")
            self.Button_TandemActionGroup.setText("串联动作文件")
            self.Button_ReSetServos.setText("舵机回中")
            self.Button_DelectSingle.setText("单个擦除")
            self.Button_AllDelect.setText("全部擦除")
            self.Button_RunAction.setText("动作运行")
            self.Button_StopAction.setText("动作停止")
            self.Button_Run.setText("运行")
            self.checkBox_time.setToolTip("锁定时间")
            self.checkBox.setText("循环")
            self.label_time.setText("时间")
            self.label_total_time.setText("运行总时间")
            self.label_action.setText("动作组")
            self.HandEnabled.setText("开合手掌")
        elif name.text() == "English":
            self.chinese = False
            self.Button_ServoPowerDown.setText("Manual")
            self.Button_AngularReadback.setText("Read angle")
            self.Button_AddAction.setText("Add action")
            self.Button_DelectAction.setText("Delete action")
            self.Button_UpdateAction.setText("Update action")
            self.Button_InsertAction.setText("Insert action")
            self.Button_MoveUpAction.setText("Action upward")
            self.Button_MoveDownAction.setText("Action down")        
            self.Button_OpenActionGroup.setText("Open action file")
            self.Button_SaveActionGroup.setText("Save action file")
            self.Button_ReadDeviation.setText("Read deviation")
            self.Button_DownloadDeviation.setText("Download deviation")
            self.Button_TandemActionGroup.setText("Integrate file")
            self.Button_ReSetServos.setText("Reset servo")
            self.Button_DelectSingle.setText("Erase single")
            self.Button_AllDelect.setText("All erase")
            self.Button_RunAction.setText("Run action")
            self.Button_StopAction.setText("Stop")
            self.Button_Run.setText("Run")           
            self.checkBox_time.setToolTip("Lock time")
            self.checkBox.setText("Loop")
            self.label_time.setText("Duration")
            self.label_total_time.setText("Total duration")  
            self.label_action.setText("Action list") 
            self.HandEnabled.setText("Hands")

    def eventFilter(self, widget, event):
        if event.type() == QEvent.KeyPress:
            if event.key()== 16777220 or event.key() == 16777221:
                self.horizontalSlider_manual = False
                for i in range(self.all_joints):
                    pulse = int(self.lineEdit_object[i].text())
                    self.horizontalSlider_pulse_object[i].setValue(pulse)
                    self.ctl.set_bus_servo_pulse(i + 1, pulse, 500)
                self.horizontalSlider_manual = True
        return False
    
    # 滑竿同步对应文本框的数值,及滑竿控制相应舵机转动(synchronize the slider with the corresponding text box value, and control the corresponding servo rotation with the slider)
    def pulse_valuechange(self, name):
        if self.horizontalSlider_manual:
            servo_pulse = self.horizontalSlider_pulse_object[name].value()
            self.ctl.set_bus_servo_pulse(name + 1, servo_pulse, self.min_time)
            self.lineEdit_object[name].setText(str(servo_pulse))            

    def dev_valuechange(self, name):
        if self.horizontalSlider_dev_manual:
            self.devNew[0] = self.horizontalSlider_dev_object[name].value()
            self.ctl.set_bus_servo_deviation(name + 1, self.devNew[0])
            self.label_object[name].setText(str(self.devNew[0]))
            if self.devNew[0] < 0:
                self.devNew[0] = 0xff + self.devNew[0] + 1

    # 复位按钮点击事件(reset button click event)
    def button_re_clicked(self, name):
        self.horizontalSlider_manual = False
        if name == 'reSetServos':
            for i in range(self.all_joints):
                self.horizontalSlider_pulse_object[i].setValue(500)
                self.ctl.set_bus_servo_pulse(i + 1, 500, 1000)
                self.lineEdit_object[i].setText('500')
            self.horizontalSlider_manual = True

    # 选项卡选择标签状态，获取对应舵机数值(tab selection label status, obtain corresponding servo values)
    def tab_index(self, index):       
        return [str(self.horizontalSlider_pulse_object[i].value()) for i in range(self.all_joints)]
    
    def getIndexData(self, index):
        return [str(self.tableWidget.item(index, j).text()) for j in range(2, self.tableWidget.columnCount())]
    
    # 往tableWidget表格添加一行数据的函数(a function to add a row of data to the tableWidget)
    def add_line(self, data):
        self.tableWidget.setItem(data[0], 1, QtWidgets.QTableWidgetItem(str(data[0] + 1)))       
        for i in range(2, len(data) + 1):          
            self.tableWidget.setItem(data[0], i, QtWidgets.QTableWidgetItem(data[i - 1]))

    # 在定位行添加运行图标按钮(add a run icon button to the designated row)
    def icon_position(self):
        self.Button_run = QtWidgets.QToolButton()
        self.Button_run.setIcon(self.icon)              
        item = self.tableWidget.currentRow()
        self.tableWidget.setCellWidget(item, 0, self.Button_run)
        for i in range(self.tableWidget.rowCount()):
            if i != item:
                self.tableWidget.removeCellWidget(i, 0)      
        self.Button_run.clicked.connect(self.action_one)
        
    def action_one(self):        
        self.horizontalSlider_manual = False
        item = self.tableWidget.currentRow()
        try:
            timer = int(self.tableWidget.item(self.tableWidget.currentRow(), 2).text())
            if not self.checkBox_time.isChecked():
                self.lineEdit_time.setText(str(timer))
            for id in range(1, self.joints + 1):
                pulse = int(self.tableWidget.item(item, id + 2).text())
                self.horizontalSlider_pulse_object[id - 1].setValue(pulse)
                self.lineEdit_object[id - 1].setText(str(pulse))
                self.ctl.set_bus_servo_pulse(id, pulse, timer)           
        except BaseException as e:
            print(e)
            if self.chinese:
                self.message_from('运行出错')
            else:
                self.message_from('Running error')
        self.horizontalSlider_manual = True

    # 编辑动作组按钮点击事件(edit action group button click event)
    def button_editaction_clicked(self, name):
        list_data = self.tab_index(self.tableWidget.currentIndex())
        RowCont = self.tableWidget.rowCount()
        item = self.tableWidget.currentRow()
        if name == 'servoPowerDown':
            for id in range(1, self.all_joints + 1):
                self.ctl.unload_bus_servo(id)
            if self.chinese:
                self.message_from('掉电成功')
            else:
                self.message_from('success')
        if name == 'angularReadback':
            self.tableWidget.insertRow(RowCont)    # 增加一行(add one row)
            self.tableWidget.selectRow(RowCont)    # 定位最后一行为选中行(locate the last row as the selected row)
            use_time = int(self.lineEdit_time.text())
            data = [RowCont, str(use_time)]
            for i in range(1, self.joints + 1):
                pulse = self.ctl.get_bus_servo_pulse(i)
                if pulse is None:
                    return
                else:
                    data.append(str(pulse))     
            if self.joints < 18:
                data.append(str(500))
                data.append(str(500))                                  
            if use_time < 20:
                if self.chinese:
                    self.message_from('运行时间必须大于20ms')
                else:
                    self.message_from('Run time must be greater than 20ms')
                return         
            self.add_line(data)
            self.totalTime += use_time
            self.label_TotalTime.setText(str((self.totalTime)/1000.0))            
        if name == 'addAction':    # 添加动作(add action)
            use_time = int(self.lineEdit_time.text())
            data = [RowCont, str(use_time)]            
            if use_time < 20:
                if self.chinese:
                    self.message_from('运行时间必须大于20ms')
                else:
                    self.message_from('Run time must be greater than 20ms')
                return
            self.tableWidget.insertRow(RowCont)    # 增加一行(add one row)
            self.tableWidget.selectRow(RowCont)    # 定位最后一行为选中行(select the last row as the selected row)
            data.extend(list_data)
            self.add_line(data)
            self.totalTime += use_time
            self.label_TotalTime.setText(str((self.totalTime)/1000.0))
        if name == 'delectAction':    # 删除动作(delete action)
            if RowCont != 0:
                self.totalTime -= int(self.tableWidget.item(item, 2).text())
                self.tableWidget.removeRow(item)  # 删除选定行(delete the selected row)
                self.label_TotalTime.setText(str((self.totalTime)/1000.0))
        if name == 'delectAllAction':
            if self.chinese:
                result = self.message_delect('此操作会删除列表中的所有动作，是否继续？')
            else:
                result = self.message_delect('This operation will delete all actions in the list. Do you want to continue?')
                
            if result == 0:                              
                for i in range(RowCont):
                    self.tableWidget.removeRow(0)
                self.totalTime = 0
                self.label_TotalTime.setText(str(self.totalTime))
            else:
                pass          
        if name == 'updateAction':    # 更新动作(update action)
            use_time = int(self.lineEdit_time.text())
            data = [item, str(use_time)]            
            if use_time < 20:
                if self.chinese:
                    self.message_from('运行时间必须大于20ms')
                else:
                    self.message_from('Run time must be greater than 20ms')
                return
            data.extend(list_data)
            self.add_line(data)
            self.totalTime = 0
            for i in range(RowCont):
                self.totalTime += int(self.tableWidget.item(i,2).text())
            self.label_TotalTime.setText(str((self.totalTime)/1000.0))
        if name == 'mirrorAction':
            list_data = self.getIndexData(item)
            use_time = int(list_data[0])
            list_data = list_data[1:]
            data = [item, str(use_time)]           
            if use_time < 20:
                if self.chinese:
                    self.message_from('运行时间必须大于20ms')
                else:
                    self.message_from('Run time must be greater than 20ms')
                return
            for i in range(int(self.joints/2)-1):
                if int(list_data[i]) + int(list_data[i + int(self.joints/2)-1]) != 1000:
                    tmp = int(list_data[i])
                    list_data[i] = str(1000 - int(list_data[i + int(self.joints/2)-1]))
                    list_data[i + int(self.joints/2)-1] = str(1000 - tmp)
            tmp = int(list_data[16])
            list_data[16] = str(1000 - int(list_data[17]))   
            list_data[17] = str(1000 - tmp)   
            data.extend(list_data)
            self.add_line(data)
        if name == 'mirrorAllAction':
            for item in range(RowCont):
                list_data = self.getIndexData(item)
                use_time = int(list_data[0])
                list_data = list_data[1:]
                data = [item, str(use_time)]           
                if use_time < 20:
                    if self.chinese:
                        self.message_from('运行时间必须大于20ms')
                    else:
                        self.message_from('Run time must be greater than 20ms')
                    return
                
                for i in range(int(self.joints/2)-1):
                    if int(list_data[i]) + int(list_data[i + int(self.joints/2)-1]) != 1000:
                        tmp = int(list_data[i])
                        list_data[i] = str(1000 - int(list_data[i + int(self.joints/2)-1]))
                        list_data[i + int(self.joints/2)-1] = str(1000 - tmp)
                tmp = int(list_data[16])
                list_data[16] = str(1000 - int(list_data[17]))   
                list_data[17] = str(1000 - tmp)  
                        
                data.extend(list_data)
                self.add_line(data)           
        if name == 'insertAction':    # 插入动作(insert action)
            if item == -1:
                return
            use_time = int(self.lineEdit_time.text())
            data = [item, str(use_time)]            
            if use_time < 20:
                if self.chinese:
                    self.message_from('运行时间必须大于20ms')
                else:
                    self.message_from('Run time must be greater than 20ms')
                return

            self.tableWidget.insertRow(item)       # 插入一行(insert one row)
            self.tableWidget.selectRow(item)
            data.extend(list_data)
            self.add_line(data)
            self.totalTime += use_time
            self.label_TotalTime.setText(str((self.totalTime)/1000.0))
        if name == 'moveUpAction':
            data_new = [item - 1]
            data = [item]
            if item == 0 or item == -1:
                return
            current_data = self.getIndexData(item)
            uplist_data = self.getIndexData(item - 1)
            data_new.extend(current_data)
            data.extend(uplist_data)
            self.add_line(data_new)           
            self.add_line(data)
            self.tableWidget.selectRow(item - 1) 
        if name == 'moveDownAction':
            data_new = [item + 1]
            data = [item]            
            if item == RowCont - 1:
                return
            current_data = self.getIndexData(item)
            downlist_data = self.getIndexData(item + 1)
            data_new.extend(current_data)
            data.extend(downlist_data)
            self.add_line(data_new)           
            self.add_line(data)            
            self.tableWidget.selectRow(item + 1)
                             
        for i in range(self.tableWidget.rowCount()):    #刷新编号值(refresh number)
            self.tableWidget.item(i , 2).setFlags(self.tableWidget.item(i , 2).flags() & ~Qt.ItemIsEditable)
            self.tableWidget.setItem(i,1,QtWidgets.QTableWidgetItem(str(i + 1)))
        self.icon_position()

    # 在线运行按钮点击事件(online run button click event)
    def button_run(self, name):
        if self.tableWidget.rowCount() == 0:
            if self.chinese:
                self.message_from('请先添加动作!')
            else:
                self.message_from('Please add action first!')
        else:
            if name == 'run':
                try:
                    if not self.running:
                        self.running = True
                        if self.Button_Run.text() == 'Run':
                            self.Button_Run.setText('Stop')
                        else:
                            self.Button_Run.setText('停止')                        
                        #self.Button_Run.setStyleSheet("QPushButton{image: url(:/images/pause-online.png);}" "QPushButton{border-radius:5px;}")
                        self.row = self.tableWidget.currentRow()
                        self.run_action_one()                       
                        if self.checkBox.isChecked():
                            self.loop = True
                        else:
                            self.loop = False
                            
                        self.timer = QTimer()                       
                        self.timer.timeout.connect(self.run_action_online)
                        self.use_time_list = []
                        for i in range(self.tableWidget.rowCount() - self.row):
                            use_time = int(self.tableWidget.item(i, 2).text())
                            self.use_time_list.append(use_time)
                        self.use_time_list_ = copy.deepcopy(self.use_time_list)
                        self.timer.start(self.use_time_list_[0])                       
                    else:
                        self.running = False
                        self.timer.stop()
                        if self.Button_Run.text() == 'Stop':
                            self.Button_Run.setText('Run')
                        else:
                            self.Button_Run.setText('运行')
                        #self.Button_Run.setStyleSheet("QPushButton{image: url(:/images/play-music.png);}" "QPushButton{border-radius:5px;}")                     
                        if self.chinese:
                            self.message_from('运行结束!')
                        else:
                            self.message_from('Run over!')
                except BaseException as e:
                    print(e)                 

    def run_action_online(self):                
        item = self.tableWidget.currentRow()
        use_time = int(self.tableWidget.item(item, 2).text())
        if item == self.tableWidget.rowCount() - 1:
            if self.loop:
                self.use_time_list_ = copy.deepcopy(self.use_time_list)
                self.tableWidget.selectRow(self.row)
                self.run_action_one()
                self.timer.start(self.use_time_list_[0])
            else:                
                self.timer.stop()
                self.running = False
                if self.Button_Run.text() == 'Stop':
                    self.Button_Run.setText('Run')
                else:
                    self.Button_Run.setText('运行')
                #self.Button_Run.setStyleSheet("QPushButton{image: url(:/images/play-music.png);}" "QPushButton{border-radius:5px;}")                
                if self.chinese:
                    self.message_from('运行结束!')
                else:
                    self.message_from('Run over!')
                self.tableWidget.selectRow(0)
        else:
            self.use_time_list_.remove(self.use_time_list_[0])
            self.timer.start(self.use_time_list_[0])            
            self.tableWidget.selectRow(item + 1)
            self.run_action_one()
                                                                                                                                               
        self.icon_position()         

    def run_action_one(self):
        try:
            data = []
            item = self.tableWidget.currentRow()
            timer = int(self.tableWidget.item(self.tableWidget.currentRow(), 2).text())
            for id in range(1, self.joints + 1):
                pulse = int(self.tableWidget.item(item, id + 2).text())
                data.extend((id, pulse))
                self.ctl.set_bus_servo_pulse(id, pulse, timer)
        except BaseException as e:
            print(e)
            if self.chinese:
                self.message_from('运行出错')
            else:
                self.message_from('Running error')

    # 文件打开及保存按钮点击事件(file open and save button click event)
    def button_flie_operate(self, name):
        try:            
            if name == 'openActionGroup':
                dig_o = QFileDialog()
                dig_o.setFileMode(QFileDialog.ExistingFile)
                dig_o.setNameFilter('d6a Flies(*.d6a)')
                openfile = dig_o.getOpenFileName(self, 'OpenFile', '/userdata/dev_ws/src/originman/driver/ActionGroups', 'd6a Flies(*.d6a)')
                # 打开单个文件(open single file)
                # 参数一：设置父组件；参数二：QFileDialog的标题(parameter 1: Set parent component; Parameter 2: Title of QFileDialog)
                # 参数三：默认打开的目录，“.”点表示程序运行目录，/表示当前盘符根目录(parameter 3: Default directory to open. '.' represents the directory where the program is running, '/' represents the root directory of the current drive)
                # 参数四：对话框的文件扩展名过滤器Filter，比如使用 Image files(*.jpg *.gif) 表示只能显示扩展名为.jpg或者.gif文件(parameter 4: Filter for file extension of the dialog box. For example, using 'Image files (*.jpg *.gif)' will only display files with the extensions .jpg or .gif)
                # 设置多个文件扩展名过滤，使用双引号隔开；“All Files(*);;PDF Files(*.pdf);;Text Files(*.txt)”(set multiple file extension filters, separated by double quotation marks; “All Files(*);;PDF Files(*.pdf);;Text Files(*.txt)”)
                path = openfile[0]
                try:
                    if path != '':
                        rbt = QSqlDatabase.addDatabase("QSQLITE")
                        rbt.setDatabaseName(path)
                        if rbt.open():
                            actgrp = QSqlQuery()
                            if (actgrp.exec("select * from ActionGroup ")):
                                self.tableWidget.setRowCount(0)
                                self.tableWidget.clearContents()
                                self.totalTime = 0
                                while (actgrp.next()):
                                    count = self.tableWidget.rowCount()
                                    self.tableWidget.setRowCount(count + 1)
                                    for i in range(self.all_joints + 2):
                                        self.tableWidget.setItem(count, i + 1, QtWidgets.QTableWidgetItem(str(actgrp.value(i))))
                                        if i == 1:
                                            self.totalTime += actgrp.value(i)
                                        self.tableWidget.update()
                                        self.tableWidget.selectRow(count)
                                    self.tableWidget.item(count , 2).setFlags(self.tableWidget.item(count , 2).flags() & ~Qt.ItemIsEditable)                        
                                               
                        rbt.close()
                        self.label_TotalTime.setText(str(self.totalTime/1000.0))
                        self.tableWidget.selectRow(0)
                        self.icon_position()                    
                except:
                    if self.chinese:
                        self.message_from('错误的动作组格式')
                    else:
                        self.message_from('Wrong action group format')
                self.label_action_number.setText(str(path))                                           
            if name == 'saveActionGroup':
                dig_s = QFileDialog()
                
                if self.tableWidget.rowCount() == 0:
                    if self.chinese:
                        self.message_from('动作列表是空的哦，没啥要保存的')
                    else:
                        self.message_from('The action list is empty, nothing to save')                      
                    return
                
                savefile = dig_s.getSaveFileName(self, 'Savefile', '', 'd6a Flies(*.d6a)')
                path = savefile[0]
                
                if os.path.isfile(path):
                    
                    os.system('sudo rm ' + path)
                if path != '':       
                                
                    if path[-4:] == '.d6a':
                        conn = sqlite3.connect(path)
                    else:
                        conn = sqlite3.connect(path + '.d6a')
                    
                    c = conn.cursor()                    
                    c.execute('''CREATE TABLE ActionGroup([Index] INTEGER PRIMARY KEY AUTOINCREMENT
                    NOT NULL ON CONFLICT FAIL
                    UNIQUE ON CONFLICT ABORT,
                    Time INT,
                    Servo1 INT,
                    Servo2 INT,
                    Servo3 INT,
                    Servo4 INT,
                    Servo5 INT,
                    Servo6 INT,
                    Servo7 INT,
                    Servo8 INT,
                    Servo9 INT,
                    Servo10 INT,
                    Servo11 INT,
                    Servo12 INT,
                    Servo13 INT,
                    Servo14 INT,
                    Servo15 INT,                    
                    Servo16 INT,
                    Servo17 INT,                    
                    Servo18 INT);''')                      
                    for i in range(self.tableWidget.rowCount()):
                        insert_sql = "INSERT INTO ActionGroup(Time, Servo1, Servo2, Servo3, Servo4, Servo5, Servo6, Servo7, Servo8, Servo9, Servo10, Servo11, Servo12, Servo13, Servo14, Servo15, Servo16, Servo17, Servo18) VALUES("
                        for j in range(2, self.tableWidget.columnCount()):
                            if j == self.tableWidget.columnCount() - 1:
                                insert_sql += str(self.tableWidget.item(i, j).text())
                            else:
                                insert_sql += str(self.tableWidget.item(i, j).text()) + ','
                        
                        insert_sql += ");"
                        c.execute(insert_sql)
                    
                    conn.commit()
                    conn.close()
                    self.reflash_action()
            if name == 'readDeviation':
                self.horizontalSlider_dev_manual = False
                id = ''
                self.readDevOk = True
                dev_data = []
                for i in range(1, self.joints + 1):
                    dev = self.ctl.get_bus_servo_deviation(i)
                    
                    if dev == 999:
                        
                        dev_data.append(0)
                        id += (' id' + str(i))
                    elif dev > 125:  # 负数(negative number)
                        
                        dev_data.append(-(0xff - (dev - 1)))                        
                    else:
                        dev_data.append(dev)
                for i in range(self.joints):
                    self.horizontalSlider_dev_object[i].setValue(dev_data[i])
                    self.label_object[i].setText(str(dev_data[i]))
                if id == '':
                    if self.chinese:
                        self.message_from('读取偏差成功!')
                    else:
                        self.message_from('success!')
                else:
                    if self.chinese:
                        self.message_from(id + '号舵机偏差读取失败!')
                    else:
                        self.message_from('Failed to read the deviation of' + id)
                self.horizontalSlider_dev_manual = True
            if name == 'downloadDeviation':
                if self.readDevOk:
                    for id in range(1, self.joints + 1):
                        self.ctl.save_bus_servo_deviation(id)
                    if self.chinese:
                        self.message_from('下载偏差成功!')
                    else:
                        self.message_from('success!')
                else:
                    if self.chinese:
                        self.message_from('请先读取偏差！')
                    else:
                        self.message_from('Please read the deviation first！')
            if name == 'tandemActionGroup':
                dig_t = QFileDialog()
                dig_t.setFileMode(QFileDialog.ExistingFile)
                dig_t.setNameFilter('d6a Flies(*.d6a)')
                openfile = dig_t.getOpenFileName(self, 'OpenFile', '', 'd6a Flies(*.d6a)')
                # 打开单个文件(open the single file)
                # 参数一：设置父组件；参数二：QFileDialog的标题(parameter 1: Set parent component; Parameter 2: Title of QFileDialog)
                # 参数三：默认打开的目录，“.”点表示程序运行目录，/表示当前盘符根目录(parameter 3: Default directory to open. '.' represents the directory where the program is running, '/' represents the root directory of the current drive)
                # 参数四：对话框的文件扩展名过滤器Filter，比如使用 Image files(*.jpg *.gif) 表示只能显示扩展名为.jpg或者.gif文件(parameter 4: Filter for file extension of the dialog box. For example, using 'Image files (*.jpg *.gif)' will only display files with the extensions .jpg or .gif)
                # 设置多个文件扩展名过滤，使用双引号隔开；“All Files(*);;PDF Files(*.pdf);;Text Files(*.txt)”(set multiple file extension filters, separated by double quotation marks)；“All Files(*);;PDF Files(*.pdf);;Text Files(*.txt)”
                path = openfile[0]
                try:
                    if path != '':
                        tbt = QSqlDatabase.addDatabase("QSQLITE")
                        tbt.setDatabaseName(path)
                        if tbt.open():
                            actgrp = QSqlQuery()
                            if (actgrp.exec("select * from ActionGroup ")):
                                while (actgrp.next()):
                                    count = self.tableWidget.rowCount()
                                    self.tableWidget.setRowCount(count + 1)
                                    for i in range(self.all_joints + 2):
                                        if i == 0:
                                            self.tableWidget.setItem(count, i + 1, QtWidgets.QTableWidgetItem(str(count + 1)))
                                        else:                      
                                            self.tableWidget.setItem(count, i + 1, QtWidgets.QTableWidgetItem(str(actgrp.value(i))))
                                        if i == 1:
                                            self.totalTime += actgrp.value(i)
                                        self.tableWidget.update()
                                        self.tableWidget.selectRow(count)
                                    self.tableWidget.item(count , 2).setFlags(self.tableWidget.item(count , 2).flags() & ~Qt.ItemIsEditable)
                        self.icon_position()
                        tbt.close()
                        self.tableWidget.selectRow(0)
                        self.label_TotalTime.setText(str(self.totalTime/1000.0))
                except:
                    if self.chinese:
                        self.message_from('错误的动作组格式')
                    else:
                        self.message_from('Wrong action group format')
        except BaseException as e:
            print(e)

    def listActions(self, path):
        if not os.path.exists(path):
            os.mkdir(path)
        pathlist = os.listdir(path)
        actList = []
        
        for f in pathlist:
            if f[0] == '.':
                pass
            else:
                if f[-4:] == '.d6a':
                    f.replace('-', '')
                    if f:
                        actList.append(f[0:-4])
                else:
                    pass
        return actList
    
    def reflash_action(self):
        actList = self.listActions(self.actdir)
        actList.sort()
        
        if len(actList) != 0:        
            self.comboBox_action.clear()
            for i in range(0, len(actList)):
                self.comboBox_action.addItem(actList[i])
        else:
            self.comboBox_action.clear()
    
    # 控制动作组按钮点击事件(control action group button click event)
    def button_controlaction_clicked(self, name):
        if name == 'delectSingle':
            if self.chinese:
                result = self.message_delect('此操作会删除当前动作组，是否继续？')
            else:
                result = self.message_delect('This operation will delete current action group. Do you want to continue？')               
            if result == 0:                              
                if str(self.comboBox_action.currentText()) != "":
                    os.remove(self.actdir + str(self.comboBox_action.currentText()) + ".d6a")            
                    self.reflash_action()
        if name == 'allDelect':
            if self.chinese:
                result = self.message_delect('此操作会删除所有动作组，是否继续？')
            else:
                result = self.message_delect('This operation will delete all action groups. Do you want to continue？')               
            if result == 0:                              
                actList = self.listActions(self.actdir)
                for d in actList:
                    os.remove(self.actdir + d + '.d6a')
            self.reflash_action()
        if name == 'runAction':   # 动作组运行(action group running)
            run_action_group(self.comboBox_action.currentText())
        if name == 'stopAction':   # 停止运行(stop running)
            stop_action_group()

if __name__ == "__main__":  
    app = QtWidgets.QApplication(sys.argv)
    myshow = MainWindow()
    myshow.setWindowTitle("OriginMan 1.20")
    myshow.show()
    sys.exit(app.exec_())
