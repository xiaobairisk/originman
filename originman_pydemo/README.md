## 文件说明

这是一个用Python实现的一个驱动demo，用于演示如何用Python使用

- IMU:每隔1s显示一次IMU数据
- 灯带：灯带彩色闪烁；
- 蜂鸣器：蜂鸣器响动三次
- 显示屏：显示IP地址；
- 相机：用opencv显示相机画面；
- 语音：语音播放、录音；

## 使用说明
### IMU

执行以下指令

```bash
python3 imu_print.py
```

将看到终端输出IMU数据：

```bash
Board initialized
Starting IMU display (press Ctrl+C to stop)
- IMU - Accel: (-0.95, 0.02, 0.02) g, Gyro: (-1.61, -3.14, 0.14) deg/s
- IMU - Accel: (-0.95, 0.02, 0.02) g, Gyro: (-1.38, -2.43, -0.06) deg/s
- IMU - Accel: (-0.95, 0.02, 0.02) g, Gyro: (-1.58, -2.80, 0.25) deg/s
- IMU - Accel: (-0.95, 0.02, 0.02) g, Gyro: (-1.42, -2.82, -0.03) deg/s
- IMU - Accel: (-0.95, 0.02, 0.02) g, Gyro: (-1.48, -2.57, 0.09) deg/s
- IMU - Accel: (-0.95, 0.02, 0.02) g, Gyro: (-1.28, -1.80, -0.22) deg/s
- IMU - Accel: (-0.95, 0.02, 0.02) g, Gyro: (-1.83, -2.69, 0.06) deg/s
- IMU - Accel: (-0.95, 0.02, 0.02) g, Gyro: (-1.40, -3.70, 0.22) deg/s
```

### 灯带
执行以下指令

```bash
python3 rgb_ring_controller.py
```

将看到OriginMan胸口的等待彩色旋转；

### 蜂鸣器
执行以下指令

```bash
python3 buzzon_on.py
```

将听到蜂鸣器响动三次；


### 显示屏
执行以下指令

```bash
python3 oled_ip_display
```

将看到OriginMan背部的显示屏显示IP地址；

### 相机
执行以下指令

```bash
python3 video_capture.py
```

将看到弹出一个界面，显示此时OriginMan相机看到的画面；

### 语音
首先进入audio文件夹

```bash
python3 test_audio.py
```

将播放prompt.wav中的音频，然后录音2s，在播放录音数据；
