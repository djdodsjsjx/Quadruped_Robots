import sensor, image, time, math, struct
import json
from pyb import LED,Timer
from struct import pack, unpack
import LineFollowing,Message
#初始化镜头
sensor.reset()
sensor.set_pixformat(sensor.RGB565)#设置相机模块的像素模式
sensor.set_framesize(sensor.QQVGA)#设置相机分辨率160*120
sensor.skip_frames(time=3000)#时钟
sensor.set_auto_whitebal(False)#若想追踪颜色则关闭白平衡
clock = time.clock()#初始化时钟

#主循环
while(True):
    clock.tick()#时钟初始化
    #线检测
    LineFollowing.LineCheck()
    #计算程序运行频率
    fps=int(clock.fps())
    T_ms = (int)(1000/fps)
    print('fps',fps,'T_ms',T_ms)
