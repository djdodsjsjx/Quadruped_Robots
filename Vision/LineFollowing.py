import sensor, image, time, math, struct
import json
import Message
#Black_threshold =(4, 31, -20, 49, -36, 58)# 寻线 用  黑色
Black_threshold =(4, 19, -17, 42, -32, 37)
rad_to_angle = 57.29#弧度转度
IMG_WIDTH = 160
IMG_HEIGHT = 120
# 取样窗口(x, y, len, wid)
ROIS = {
    'down':   (5, 100, 150, 15), # 横向取样-下方       1
    'middle': (5, 52,  150, 15), # 横向取样-中间       2
    'up':     (5,  5,  150, 15), # 横向取样-上方       3
    'left':   (5,  5,  15, 110), # 纵向取样-左侧       4
    'right':  (140,5,  15, 110), # 纵向取样-右侧       5
    'All':    (5,  5,  150,110), # 全画面取样-全画面    6
}
class Line(object):
    flag = 0
    color = 0
    angle = 0
    distance = 0
    cross_x=0
    cross_y=0
    cross_flag=0

class LineFlag(object):
    turn_left = 0
    turn_right = 0

LineFlag=LineFlag()
Line=Line()
# y = k * x + b
def CalculateIntersection(line1, line2):
    # c1, c2: 直线的截距b
    a1 = line1.y2() - line1.y1()
    b1 = line1.x1() - line1.x2()
    c1 = line1.x2()*line1.y1() - line1.x1()*line1.y2()

    a2 = line2.y2() - line2.y1()
    b2 = line2.x1() - line2.x2()
    c2 = line2.x2() * line2.y1() - line2.x1()*line2.y2()
    # 若两条直线不平行，求出交点
    if (a1 * b2 - a2 * b1) != 0 and (a2 * b1 - a1 * b2) != 0:
        cross_x = int((b1*c2-b2*c1)/(a1*b2-a2*b1))
        cross_y = int((c1*a2-c2*a1)/(a1*b2-a2*b1))

        Line.cross_flag = 1
        Line.cross_x = cross_x-80
        Line.cross_y = cross_y-60
        img.draw_cross(cross_x,cross_y,5,color=[255,0,0])
        return (cross_x, cross_y)
    else:
        Line.cross_flag = 0
        Line.cross_x = 0
        Line.cross_y = 0
        return None
def calculate_angle(line1, line2):
    '''
    利用四边形的角公式， 计算出直线夹角
    '''
    angle  = (180 - abs(line1.theta() - line2.theta()))
    if angle > 90:
        angle = 180 - angle
    return angle
def find_interserct_lines(lines, angle_threshold=(10,90), window_size=None):
    '''
    根据夹角阈值寻找两个相互交叉的直线， 且交点需要存在于画面中
    '''
    line_num = len(lines)
    for i in range(line_num -1):
        for j in range(i, line_num):
            # 判断两个直线之间的夹角是否为直角
            angle = calculate_angle(lines[i], lines[j])
            # 判断角度是否在阈值范围内
            if not(angle >= angle_threshold[0] and angle <=  angle_threshold[1]):
                continue

            # 判断交点是否在画面内
            if window_size is not None:
                # 获取窗口的尺寸 宽度跟高度
                win_width, win_height = window_size
                # 获取直线交点
                intersect_pt = CalculateIntersection(lines[i], lines[j])
                if intersect_pt is None:
                    # 没有交点
                    Line.cross_x = 0
                    Line.cross_y = 0
                    Line.cross_flag = 0
                    continue
                x, y = intersect_pt
                if not(x >= 0 and x < win_width and y >= 0 and y < win_height):
                    # 交点如果没有在画面中
                    Line.cross_x = 0
                    Line.cross_y = 0
                    Line.cross_flag = 0
                    continue
            return (lines[i], lines[j])
    return None


#寻找每个感兴趣区里的指定色块并判断是否存在
def find_blobs_in_rois(img):
    '''
    在ROIS中寻找色块,获取ROI中色块的中心区域与是否有色块的信息
    '''
    global ROIS

    roi_blobs_result = {}  # 在各个ROI中寻找色块的结果记录
    for roi_direct in ROIS.keys():#数值复位
        roi_blobs_result[roi_direct] = {
            'cx': -1,
            'cy': -1,
            'blob_flag': False
        }
    for roi_direct, roi in ROIS.items():
        blobs=img.find_blobs([Black_threshold], roi=roi, merge=True, pixels_area=100)
        if len(blobs) == 0:
            continue

        largest_blob = max(blobs, key=lambda b: b.pixels())
        x,y,width,height = largest_blob[:4]

        if not(width >= 15 and width <= 70 and height >= 15 and height <= 30):
            # 根据色块的长宽进行过滤
            continue

        roi_blobs_result[roi_direct]['cx'] = largest_blob.cx()
        roi_blobs_result[roi_direct]['cy'] = largest_blob.cy()
        roi_blobs_result[roi_direct]['blob_flag'] = True
        img.draw_rectangle((x,y,width, height), color=(0,255,255))

    CX1 = roi_blobs_result['up']['cx']
    CX2 = roi_blobs_result['middle']['cx']
    CX3 = roi_blobs_result['down']['cx']
    CY1 = roi_blobs_result['up']['cy']
    CY2 = roi_blobs_result['middle']['cy']
    CY3 = roi_blobs_result['down']['cy']
    up_flag = roi_blobs_result['up']['blob_flag']
    middle_flag = roi_blobs_result['middle']['blob_flag']
    down_flag = roi_blobs_result['down']['blob_flag']
    left_flag = roi_blobs_result['left']['blob_flag']
    right_flag = roi_blobs_result['right']['blob_flag']

    if (up_flag and middle_flag and down_flag and not left_flag and not right_flag): 
        angle12 = int(math.atan((CX1 - CX2) / (CY1 - CY2)) * rad_to_angle)
        angle23 = int(math.atan((CX2 - CX3) / (CY2 - CY3)) * rad_to_angle)
        Line.distance = CX2 - 80
        Line.angle = angle12
        if (angle12 * angle23 > 0) :       #直线状态①
            Line.flag = 1
        elif (angle12 < 0 and angle23 > 0) :     #右转状态②
            Line.flag = 3
        elif (angle12 > 0 and angle23 < 0) :     #左转状态②
            Line.flag = 2
    if (not up_flag and down_flag and middle_flag and left_flag != right_flag):   
        if Line.cross_flag == 1:           #转向状态①
            Line.distance = 0
            Line.angle = 0
            if left_flag :
                Line.flag = 2
            if right_flag :
                Line.flag = 3
        if Line.cross_flag == 0:           #直线状态②
                Line.flag = 1
                Line.distance = CX2 - 80
                Line.angle = int(math.atan((CX2 - CX3) / (CY2 - CY3)) * rad_to_angle)
    if (not down_flag and up_flag and middle_flag and left_flag != right_flag):    #直线状态③
        Line.flag = 1
        Line.distance = CX2 - 80
        Line.angle = int(math.atan((CX1 - CX2) / (CY1 - CY2)) * rad_to_angle)



#线检测
def LineCheck():
    # 拍摄图片
    global img
    img = sensor.snapshot()
    lines = img.find_lines(threshold=1000, theta_margin = 50, rho_margin = 50)
    if not lines:
        Line.cross_x=Line.cross_y= Line.cross_flag=0
    # 寻找相交的点 要求满足角度阈值
    find_interserct_lines(lines, angle_threshold=(45,90), window_size=(IMG_WIDTH, IMG_HEIGHT))
    find_blobs_in_rois(img)
    print('交点坐标',Line.cross_x,-Line.cross_y, Line.cross_flag)
    #寻线数据打包发送
    Message.UartSendData(Message.LineDataPack(Line.flag,Line.angle,Line.distance,Line.cross_flag,Line.cross_x,Line.cross_y,Message.Ctr.T_ms))
    return Line.flag
