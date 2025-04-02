#!/usr/bin/env python3
# coding=utf8

'''
 * 只能识别汉字，将要识别的汉字转换成拼音字母，每个汉字之间空格隔开，比如：幻尔科技 --> huan er ke ji(only able to recognize Chinese characters. Convert the Chinese characters to Pinyin letters, separated by spaces. For example: 幻尔科技 --> huan er ke ji")
 * 最多添加50个词条，每个词条最长为79个字符，每个词条最多10个汉字(you can add up to 50 entries. Each entry can have a maximum of 79 characters, with a limit of 10 Chinese characters per entry)
 * 每个词条都对应一个识别号（1~255随意设置）不同的语音词条可以对应同一个识别号，(each entry corresponds to a recognition number (1 to 255, arbitrarily set). Different speech entries can correspond to the same recognition number)
 * 比如“幻尔科技”和“幻尔”都可以将识别号设置为同一个值(for example, both "幻尔科技" and "幻尔" can be assigned the same recognition number)
 * 模块上的STA状态灯：亮起表示正在识别语音，灭掉表示不会识别语音，当识别到语音时状态灯会变暗，或闪烁，等待读取后会恢复当前的状态指示(the STA status light on the module: Illuminated indicates speech recognition in progress, off indicates no speech recognition.
 When speech is recognized, the status light may dim or blink. After reading, it will revert to the current status indication)
'''

import smbus
import time
import numpy

#幻尔科技语音识别模块例程#(hiwonder voice recognition module routine)

class ASR:

    address = 0x79
    bus = None
    
    ASR_RESULT_ADDR = 100
    #识别结果存放处，通过不断读取此地址的值判断是否识别到语音，不同的值对应不同的语音(the recognition results are stored at a specific address, continuously reading the value at this address determines whether speech has been recognized. Different values correspond to different speech)

    ASR_WORDS_ERASE_ADDR = 101
    #擦除所有词条(erase all entries)

    ASR_MODE_ADDR = 102
    #识别模式设置，值范围1~3(recognition mode setting, value range 1 to 3)
    #1：循环识别模式。状态灯常亮（默认模式）(cycle recognition mode, state light constantly on (default mode))
    #2：口令模式，以第一个词条为口令。状态灯常灭，当识别到口令词时常亮，等待识别到新的语音,并且读取识别结果后即灭掉(password mode, the first entry is set as the passphrase. Status light constantly off. When the passphrase is recognized, it remains constantly on, waiting for new speech recognition. Once the recognition result is read, it turns off)
    #3：按键模式，按下开始识别，不按不识别。支持掉电保存。状态灯随按键按下而亮起，不按不亮(button mode, press to start recognition, release to stop. Support for power-off saving. The status light turns on when the button is pressed and remains off when it's released)

    ASR_ADD_WORDS_ADDR = 160
    #词条添加的地址，支持掉电保存(the address for adding entries, supports power-off saving)

    def __init__(self, bus=1):
        self.bus = smbus.SMBus(bus)
        
    def readByte(self):
        try:
            result = self.bus.read_byte(self.address)
        except:
            return None
        return result

    def writeByte(self, val):
        try:
            value = self.bus.write_byte(self.address, val)
        except:
            return False
        if value != 0:
            return False
        return True
    
    def writeData(self, reg, val):
        try:
            self.bus.write_byte(self.address,  reg)
            self.bus.write_byte(self.address,  val)
        except:
            pass

    def getResult(self):
        if ASR.writeByte(self, self.ASR_RESULT_ADDR):
            return -1        
        try:
            value = self.bus.read_byte(self.address)
        except:
            return None
        return value

    '''
    * 添加词条函数，(add entry function)
    * idNum：词条对应的识别号，1~255随意设置。识别到该号码对应的词条语音时，
    *        会将识别号存放到asr_result_address处，等待主机读取，读取后清0(the identification number corresponding to the entry can be arbitrarily set from 1 to 255. When the speech corresponding to that number is recognized,
* the identification number will be stored at the address specified by asr_result_address, awaiting retrieval by the host. After retrieval, it will be cleared to 0)
    * words：要识别汉字词条的拼音，汉字之间用空格隔开(to recognize the pinyin of Chinese character entries, spaces are used to separate the Chinese characters)
    * 
    * 执行该函数，词条是自动往后排队添加的。(execute this function, and entries are automatically added to the queue as they come)   
    '''
    def addWords(self, idNum, words):
        buf = [idNum]       
        for i in range(0, len(words)):
            buf.append(eval(hex(ord(words[i]))))
        try:
            self.bus.write_i2c_block_data(self.address, self.ASR_ADD_WORDS_ADDR, buf)
        except:
            pass
        time.sleep(0.1)
        
    def eraseWords(self):
        try:
            result = self.bus.write_byte_data(self.address, self.ASR_WORDS_ERASE_ADDR, 0)
        except:
            return False
        time.sleep(0.1)
        if result != 0:
           return False
        return True
    
    def setMode(self, mode): 
        try:
            result = self.bus.write_byte_data(self.address, self.ASR_MODE_ADDR, mode)
        except:
            return False
        time.sleep(0.1)
        if result != 0:
           return False
        return True
        
if __name__ == "__main__":
    asr = ASR()

    #添加的词条和识别模式是可以掉电保存的，第一次设置完成后，可以将1改为0(the added entries and recognition patterns can be saved even when power is lost. After the initial setup, you can change 1 to 0)
    if 1:
        asr.eraseWords()
        asr.setMode(2)
        asr.addWords(1, 'kai shi')
        asr.addWords(2, 'wang qian zou')
        asr.addWords(2, 'qian jin')
        asr.addWords(4, 'zhi zou')
        asr.addWords(2, 'wang hou tui')
        asr.addWords(3, 'wang zuo yi')
        asr.addWords(4, 'wang you yi')
    while 1:
        data = asr.getResult()
        if data:
            print("result:", data)
        elif data is None:
            print('Sensor not connected!')
            break