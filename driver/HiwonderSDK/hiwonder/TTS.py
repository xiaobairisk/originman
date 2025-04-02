#!/usr/bin/env python3
# coding=utf8
import time
import smbus  #调用树莓派IIC库


# 幻尔科技语音合成模块使用例程#(hiwonder voice synthesis module use routine)
# 使用前需要将iic波特率设置为40000(the I2C baud rate needs to be set to 40000 before use)
# 在/boot/config.txt 最后一行添加dtparam=i2c1_baudrate=40000(add "dtparam=i2c1_baudrate=40000" to the last line of /boot/config.txt)
# 如果已经添加过可跳过，然后重启即可(if it's already added, you can skip this step and simply reboot)

class TTS:
      
    address = 0x40
    bus = None

    def __init__(self, bus=1):
        self.bus = smbus.SMBus(bus)
    
    def WireReadTTSDataByte(self):
        try:
            val = self.bus.read_byte(self.address)
        except:
            return False
        return True
    
    def TTSModuleSpeak(self, sign, words):
        head = [0xFD,0x00,0x00,0x01,0x00]             #文本播放命令(the command for text-to-speech playback)
        words_list = words.encode("gb2312")            #将文本编码格式设为GB2312(set the text encoding format to GB2312)
        sign_data = sign.encode("gb2312")    
        length = len(sign_data) + len(words_list) + 2
        head[1] = length >> 8
        head[2] = length
        head.extend(list(sign_data))
        head.extend(list(words_list))       
        try:
            self.bus.write_i2c_block_data(self.address, 0, head) #向从机发送数据(send data to the slave device)
        except:
            print('Sensor not connected!')
        time.sleep(0.05)
        
if __name__ == '__main__':
    v = TTS()
    #[h0]设置单词发音方式，0为自动判断单词发音方式，1为字母发音方式，2为单词发音方式([h0]set the pronunciation mode for words, where 0 indicates automatic detection, 1 indicates letter-by-letter pronunciation, and 2 indicates word-by-word pronunciation)
    #[v10]设置音量，音量范围为0-10,10为最大音量。([v10]set the volume, with a range from 0 to 10, where 10 represents the maximum volume)
    #更多方法请参考手册(for more methods, please refer to the manual)
    v.TTSModuleSpeak("[h0][v10]","你好")   
    #注意括号里的字符长度不能超过32,如果超过了请分多次来说(please note that the length of characters inside parentheses cannot exceed 32. If it exceeds, please split it into multiple commands)
    time.sleep(1) #要适当延时等说话完成(ensure appropriate delay for speech completion)
