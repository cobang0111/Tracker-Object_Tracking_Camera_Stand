
import platform
import numpy as np
import argparse
import cv2
import serial
import time
import sys
from threading import Thread
import csv
import math


X_255_point = 0
Y_255_point = 0
X_Size = 0
Y_Size = 0
Area = 0
Angle = 0

#-----------------------------------------------

Top_name = 'mini CTS5 setting'

hsv_Lower = 0

hsv_Upper = 0


hsv_Lower0 = 0

hsv_Upper0 = 0


hsv_Lower1 = 0

hsv_Upper1 = 0


#----------- 

color_num = [   0,  1,  2,  3,  4]


h_max =     [ 255, 65,196,111,110]

h_min =     [  55,  0,158, 59, 74]


s_max =     [ 162,200,223,110,255]

s_min =     [ 114,140,150, 51,133]


v_max =     [  77,151,239,156,255]

v_min =     [   0,95,104, 61,104]


min_area =  [  50, 50, 50, 10, 10]

now_color = 0

Temp_count = 0

Read_RX =  0

mx,my = 0,0

threading_Time = 5/1000.

Config_File_Name ='Cts5_v1.dat'

#---------------------------------------------------------

# CONSTANT

SIZE_CONSTANT = 5.0
DIST_CONSTANT = 0.16
PIXEL_X_EPSILON = 20
PIXEL_Y_EPSILON = 20

# 사용자 global 변수
    
#-----------------------------------------------

def nothing(x):
    pass

#-----------------------------------------------

def create_blank(width, height, rgb_color=(0, 0, 0)):
    image = np.zeros((height, width, 3), np.uint8)
    color = tuple(reversed(rgb_color))
    image[:] = color
    return image

#-----------------------------------------------

def draw_str2(dst, target, s):
    x, y = target
    cv2.putText(dst, s, (x+1, y+1), cv2.FONT_HERSHEY_PLAIN, 0.8, (0, 0, 0), thickness = 2, lineType=cv2.LINE_AA)
    cv2.putText(dst, s, (x, y), cv2.FONT_HERSHEY_PLAIN, 0.8, (255, 255, 255), lineType=cv2.LINE_AA)

#-----------------------------------------------

def draw_str3(dst, target, s):
    x, y = target
    cv2.putText(dst, s, (x+1, y+1), cv2.FONT_HERSHEY_PLAIN, 1.5, (0, 0, 0), thickness = 2, lineType=cv2.LINE_AA)
    cv2.putText(dst, s, (x, y), cv2.FONT_HERSHEY_PLAIN, 1.5, (255, 255, 255), lineType=cv2.LINE_AA)

#-----------------------------------------------

def draw_str_height(dst, target, s, height):
    x, y = target
    cv2.putText(dst, s, (x+1, y+1), cv2.FONT_HERSHEY_PLAIN, height, (0, 0, 0), thickness = 2, lineType=cv2.LINE_AA)
    cv2.putText(dst, s, (x, y), cv2.FONT_HERSHEY_PLAIN, height, (255, 255, 255), lineType=cv2.LINE_AA)

#-----------------------------------------------

def clock():
    return cv2.getTickCount() / cv2.getTickFrequency()

#-----------------------------------------------

def Trackbar_change(now_color):
    global  hsv_Lower,  hsv_Upper
    hsv_Lower = (h_min[now_color], s_min[now_color], v_min[now_color])
    hsv_Upper = (h_max[now_color], s_max[now_color], v_max[now_color])

#-----------------------------------------------

def Hmax_change(a):
    h_max[now_color] = cv2.getTrackbarPos('Hmax', Top_name)
    Trackbar_change(now_color)

#-----------------------------------------------

def Hmin_change(a):
    h_min[now_color] = cv2.getTrackbarPos('Hmin', Top_name)
    Trackbar_change(now_color)

#-----------------------------------------------

def Smax_change(a):
    s_max[now_color] = cv2.getTrackbarPos('Smax', Top_name)
    Trackbar_change(now_color)

#-----------------------------------------------

def Smin_change(a):
    s_min[now_color] = cv2.getTrackbarPos('Smin', Top_name)
    Trackbar_change(now_color)

#-----------------------------------------------

def Vmax_change(a):
    v_max[now_color] = cv2.getTrackbarPos('Vmax', Top_name)
    Trackbar_change(now_color)

#-----------------------------------------------

def Vmin_change(a):
    v_min[now_color] = cv2.getTrackbarPos('Vmin', Top_name)
    Trackbar_change(now_color)

#-----------------------------------------------

def min_area_change(a):
    min_area[now_color] = cv2.getTrackbarPos('Min_Area', Top_name)
    if min_area[now_color] == 0:
        min_area[now_color] = 1
        cv2.setTrackbarPos('Min_Area', Top_name, min_area[now_color])
    Trackbar_change(now_color)

#-----------------------------------------------

def Color_num_change(a):
    global now_color, hsv_Lower,  hsv_Upper
    now_color = cv2.getTrackbarPos('Color_num', Top_name)
    cv2.setTrackbarPos('Hmax', Top_name, h_max[now_color])
    cv2.setTrackbarPos('Hmin', Top_name, h_min[now_color])
    cv2.setTrackbarPos('Smax', Top_name, s_max[now_color])
    cv2.setTrackbarPos('Smin', Top_name, s_min[now_color])
    cv2.setTrackbarPos('Vmax', Top_name, v_max[now_color])
    cv2.setTrackbarPos('Vmin', Top_name, v_min[now_color])
    cv2.setTrackbarPos('Min_Area', Top_name, min_area[now_color])
    hsv_Lower = (h_min[now_color], s_min[now_color], v_min[now_color])
    hsv_Upper = (h_max[now_color], s_max[now_color], v_max[now_color])

#----------------------------------------------- 


#-----------------------------------------------

#*************************

# mouse callback function

def mouse_move(event,x,y,flags,param):
    global mx, my
    if event == cv2.EVENT_MOUSEMOVE:
        mx, my = x, y


            
# *************************

def GetLengthTwoPoints(XY_Point1, XY_Point2):
    return math.sqrt( (XY_Point2[0] - XY_Point1[0])**2 + (XY_Point2[1] - XY_Point1[1])**2 )

# *************************

def FYtand(dec_val_v ,dec_val_h):
    return ( math.atan2(dec_val_v, dec_val_y) * (180.0 / math.pi))

# *************************

#degree 값을 라디안 값으로 변환하는 함수

def FYrtd(rad_val ):
    return  (rad_val * (180.0 / math.pi))


# *************************

# 라디안값을 degree 값으로 변환하는 함수

def FYdtr(dec_val):
    return  (dec_val / 180.0 * math.pi)

# *************************

def GetAngleTwoPoints(XY_Point1, XY_Point2):
    xDiff = XY_Point2[0] - XY_Point1[0]
    yDiff = XY_Point2[1] - XY_Point1[1]
    cal = math.degrees(math.atan2(yDiff, xDiff)) + 90

    if cal > 90:
        cal =  cal - 180
    return  cal

# *************************


# ************************

def hsv_setting_save():
    global Config_File_Name, color_num
    global color_num, h_max, h_min 
    global s_max, s_min, v_max, v_min, min_area

    try:
    #if 1:
        saveFile = open(Config_File_Name, 'w')
        i = 0
        color_cnt = len(color_num)

        while i < color_cnt:
            text = str(color_num[i]) + ","
            text = text + str(h_max[i]) + "," + str(h_min[i]) + ","
            text = text + str(s_max[i]) + "," + str(s_min[i]) + ","
            text = text + str(v_max[i]) + "," + str(v_min[i]) + ","
            text = text + str(min_area[i])  + "\n"
            saveFile.writelines(text)
            i = i + 1

        saveFile.close()
        print("hsv_setting_save OK")
        return 1

    except:
        print("hsv_setting_save Error~")
        return 0


#************************

def hsv_setting_read():
    global Config_File_Name
    global color_num, h_max, h_min 
    global s_max, s_min, v_max, v_min, min_area

    #try:
    if 1:
        with open(Config_File_Name) as csvfile:
            readCSV = csv.reader(csvfile, delimiter=',')
            i = 0

            for row in readCSV:
                color_num[i] = int(row[0])
                h_max[i] = int(row[1])
                h_min[i] = int(row[2])
                s_max[i] = int(row[3])
                s_min[i] = int(row[4])
                v_max[i] = int(row[5])
                v_min[i] = int(row[6])
                min_area[i] = int(row[7])
                i = i + 1

        csvfile.close()
        print("hsv_setting_read OK")
        return 1

    #except:
    #    print("hsv_setting_read Error~")
    #    return 0

    
# ******************** sg CUSTOMIZE FUNCTION *********************

# 물체를 좌우 타겟 위치에 놓기 위해 로봇 동작을 단 한 번 조절하는 함수
def obj_x_centering(obj_x_center, robot_flag):

    if obj_x_center < ((W_View_size)/2 * 0.95) :
        print("object_x_center = ", obj_x_center, "-> Need to turn left!")
        #TX_data(serial_port, 4)
        #time.sleep(3)
    elif obj_x_center > ((W_View_size)/2 * 1.05) :
        print("object_x_center = ", obj_x_center, "-> Need to turn right!")
        #TX_data(serial_port, 6)
        #time.sleep(3)
    else :
        print("robot already satisfy the standard!")
        robot_flag = 1 

    return robot_flag


#물체를 화면 y축 가운데 놓기 위해 로봇 헤드 각도를 한 번 조절하는 함수
def obj_y_centering(obj_y_center, head_flag):
    # Head position control
    if obj_y_center > (H_View_size)/2 * 1.15 :
        print("object_y_center = ", obj_y_center, "-> Need to head down!")

        #TX_data(serial_port, head_serial[cur_theta_index])
        #time.sleep(3)
    elif obj_y_center < (H_View_size)/2 * 0.85:
        print("object_y_center = ", obj_y_center, "-> Need to head up!")
        #TX_data(serial_port, head_serial[cur_theta_index])
        #time.sleep(3)
    else:
        print("Head already satisfy the standard!")
        head_flag = 1 # Mean head degree satisfied

    return head_flag

    
# **************************************************

# **************************************************

# **************************************************

if __name__ == '__main__':



    #-------------------------------------

    print ("-------------------------------------")

    print ("(2020-1-20) mini CTS5 Program.  MINIROBOT Corp.")

    print ("-------------------------------------")

    print ("")

    os_version = platform.platform()

    print (" ---> OS " + os_version)

    python_version = ".".join(map(str, sys.version_info[:3]))

    print (" ---> Python " + python_version)

    opencv_version = cv2.__version__

    print (" ---> OpenCV  " + opencv_version)   

    #-------------------------------------

    #---- user Setting -------------------

    #-------------------------------------

    W_View_size = 800 #320  #320  #640
    H_View_size = 500
    #H_View_size = int(W_View_size / 1.777)
    #H_View_size = 600  #int(W_View_size / 1.333)
    
    BPS =  4800  # 4800,9600,14400, 19200,28800, 57600, 115200
    serial_use = 1
    View_select = 0
    
    #-------------------------------------
    
    now_color = 0 #0 = Ball , 1 = Flag
    f_count = 0 # frame count variable
    
    #-------------------------------------

    print(" ---> Camera View: " + str(W_View_size) + " x " + str(H_View_size) )
    print ("")
    print ("-------------------------------------")

    #-------------------------------------

    try:
        hsv_setting_read()

    except:
        hsv_setting_save()

    #-------------------------------------

    ap = argparse.ArgumentParser()
    ap.add_argument("-v", "--video",
                    help="path to the (optional) video file")
    ap.add_argument("-b", "--buffer", type=int, default=64,
                    help="max buffer size")
    args = vars(ap.parse_args())
    img = create_blank(320, 100, rgb_color=(0, 0, 255))
    cv2.namedWindow(Top_name)
    cv2.moveWindow(Top_name,0,0)
    cv2.createTrackbar('Hmax', Top_name, h_max[now_color], 255, Hmax_change)
    cv2.createTrackbar('Hmin', Top_name, h_min[now_color], 255, Hmin_change)
    cv2.createTrackbar('Smax', Top_name, s_max[now_color], 255, Smax_change)
    cv2.createTrackbar('Smin', Top_name, s_min[now_color], 255, Smin_change)
    cv2.createTrackbar('Vmax', Top_name, v_max[now_color], 255, Vmax_change)
    cv2.createTrackbar('Vmin', Top_name, v_min[now_color], 255, Vmin_change)
    cv2.createTrackbar('Min_Area', Top_name, min_area[now_color], 255, min_area_change)
    cv2.createTrackbar('Color_num', Top_name,color_num[now_color], 4, Color_num_change)

    Trackbar_change(now_color)

    draw_str3(img, (15, 25), 'MINIROBOT Corp.')
    draw_str2(img, (15, 45), 'space: Fast <=> Video and Mask.')
    draw_str2(img, (15, 65), 's, S: Setting File Save')
    draw_str2(img, (15, 85), 'Esc: Program Exit')

    cv2.imshow(Top_name, img)

    #---------------------------

    if not args.get("video", False):
        camera = cv2.VideoCapture(0)

    else:
        camera = cv2.VideoCapture(args["video"])

    #---------------------------

    camera.set(3, W_View_size)
    camera.set(4, H_View_size)
    camera.set(5, 60)
    time.sleep(0.5)

    #---------------------------

    (grabbed, frame) = camera.read()
    draw_str2(frame, (5, 15), 'X_Center x Y_Center =  Area' )
    draw_str2(frame, (5, H_View_size - 5), 'View: %.1d x %.1d.  Space: Fast <=> Video and Mask.'
                      % (W_View_size, H_View_size))
    draw_str_height(frame, (5, int(H_View_size/2)), 'Fast operation...', 3.0 )
    mask = frame.copy()
    cv2.imshow('mini CTS5 - Video', frame )
    cv2.imshow('mini CTS5 - Mask', mask)
    cv2.moveWindow('mini CTS5 - Mask',322 + W_View_size,36)
    cv2.moveWindow('mini CTS5 - Video',322,36)
    cv2.setMouseCallback('mini CTS5 - Video', mouse_move)

    #---------------------------
    old_time = clock()

    View_select = 0
    msg_one_view = 0

    # -------- Main Loop Start --------

    detect_count = 0
    detect_count_flag = 0
    detect_count_2 = 0
    detect_count_3 = 0
    detect_count_4 = 0
    non_detect_count = 0

    head_condition = 0
    robot_condition = 0
    
    head_dir = 0
    ball_theta = 0
    ball_theta_index = -1
    
    shot_flag = 0 # Save the shot info
    shot_turn_flag = 0
    same_flag = 0
    far_flag = 0
    flag_detected = 0

    ball_flag = 0 # Save the ball position
    flag_flag = 0 # Save the initial flag position 

    direction_flag = 0 # Determine robot walking direction
    
    new = 1
    ball = [-1, -1, -1, -1] #ball pixel location
    flag = [-1, -1, -1, -1] #flag pixel location
    flag_x_center = 0 
    flag_y_center = 0 
    ball_x_center = 0
    ball_y_center = 0
    
    # MAIN LOOP
    while True:

        # grab the current frame
        (grabbed, frame) = camera.read()
        if args.get("video") and not grabbed:
            break

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2YUV)  # HSV => YUV
        # Mask using now_color
        # Not affect to real
        mask = cv2.inRange(hsv, hsv_Lower, hsv_Upper)
        hsv_Lower = (h_min[now_color], s_min[now_color], v_min[now_color])
        hsv_Upper = (h_max[now_color], s_max[now_color], v_max[now_color])
        
        # mask0 -> ball mask
        mask0 = cv2.inRange(hsv, (h_min[0], s_min[0], v_min[0]), (h_max[0], s_max[0], v_max[0]))
        # mask1 -> flag mask
        mask1 = cv2.inRange(hsv, (h_min[1], s_min[1], v_min[1]), (h_max[1], s_max[1], v_max[1]))
        '''
        mask2 = cv2.inRange(hsv, (h_min[2], s_min[2], v_min[2]), (h_max[2], s_max[2], v_max[2]))
        mask3 = cv2.inRange(hsv, (h_min[3], s_min[3], v_min[3]), (h_max[3], s_max[3], v_max[3]))
        mask4 = cv2.inRange(hsv, (h_min[4], s_min[4], v_min[4]), (h_max[4], s_max[4], v_max[4]))
        '''

        mask = cv2.erode(mask, None, iterations=1)
        mask = cv2.dilate(mask, None, iterations=1)
        mask0 = cv2.erode(mask0, None, iterations=1)
        mask0 = cv2.dilate(mask0, None, iterations=1)
        mask1 = cv2.erode(mask1, None, iterations=1)
        mask1 = cv2.dilate(mask1, None, iterations=1)

        #mask = cv2.GaussianBlur(mask, (5, 5), 2)  # softly

        # Detecting
        #cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
        cnts0 = cv2.findContours(mask0.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
        cnts1 = cv2.findContours(mask1.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
        '''
        cnts2 = cv2.findContours(mask2.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
        cnts3 = cv2.findContours(mask3.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
        cnts4 = cv2.findContours(mask4.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
        '''
        center = None

        # Ball detected
        if len(cnts0) > 0:
            c0 = max(cnts0, key=cv2.contourArea)
            #((X, Y), radius) = cv2.minEnclosingCircle(c0)

            Area0 = cv2.contourArea(c0) / min_area[0]
            if Area0 > 255:
                Area0 = 255

            if Area0 > min_area[0]:
                detect_count += 1 

                x0, y0, w0, h0 = cv2.boundingRect(c0)
                
                cv2.rectangle(frame, (x0, y0), (x0 + w0, y0 + h0), (0, 0, 255), 2)
                
        # nothing detected        
        if len(cnts0) <= 0:
            x = 0
            y = 0
            X_255_point = 0
            Y_255_point = 0
            X_Size = 0
            Y_Size = 0
            Area0 = 0
            Area1 = 0
            Angle = 0
            non_detect_count += 1



                time.sleep(3)



        if detect_count > 50 and not robot_condition:
            detect_count = 0
            ball_x_center = x0 + w0/2
            ball_y_center = y0 + h0/2
            robot_condition = obj_x_centering(ball_x_center, robot_condition)

        # Ready to move in front but distance not calculated
        # 조건 : 공 감지, 일렬 정렬, ball x center -> 결과 : y centering 후 거리 계산 후 shot 또는 직진 보행
        elif detect_count > 50 and robot_condition and not head_condition:
            detect_count = 0

            ball_x_center = x0 + w0/2
            ball_y_center = y0 + h0/2
            head_condition = obj_y_centering(ball_y_center, head_condition)

            
        
        elif non_detect_count > 50:
            detect_count = 0
            non_detect_count = 0
            print("nothing detected")
            

        Frame_time = (clock() - old_time) * 1000.

        old_time = clock()

        f_count += 1
           

        if View_select == 0 and f_count >= 20: # Fast operation 

            print(" " + str(W_View_size) + " x " + str(H_View_size) + " =  %.1f fps" % (1000/Frame_time))
            f_count = 0
            #temp = Read_RX

            pass

            

        elif View_select == 1: # Debug

            

            if msg_one_view > 0:

                msg_one_view = msg_one_view + 1

                cv2.putText(frame, "SAVE!", (50, int(H_View_size / 2)),

                            cv2.FONT_HERSHEY_PLAIN, 5, (255, 255, 255), thickness=5)

                

                if msg_one_view > 10:

                    msg_one_view = 0                

                                

            draw_str2(frame, (3, 15), 'X: %.1d, Y: %.1d, Area: %.1d' % (X_255_point, Y_255_point, Area))

            draw_str2(frame, (3, H_View_size - 5), 'View: %.1d x %.1d Time: %.1f ms  Space: Fast <=> Video and Mask.'

                      % (W_View_size, H_View_size, Frame_time))

                      

            #------mouse pixel hsv -------------------------------

            mx2 = mx

            my2 = my

            if mx2 < W_View_size and my2 < H_View_size:

                pixel = hsv[my2, mx2]

                set_H = pixel[0]

                set_S = pixel[1]

                set_V = pixel[2]

                pixel2 = frame[my2, mx2]

                if my2 < (H_View_size / 2):

                    if mx2 < (W_View_size / 2):

                        x_p = -30

                    elif mx2 > (W_View_size / 2):

                        x_p = 60

                    else:

                        x_p = 30

                    draw_str2(frame, (mx2 - x_p, my2 + 15), '-HSV-')

                    draw_str2(frame, (mx2 - x_p, my2 + 30), '%.1d' % (pixel[0]))

                    draw_str2(frame, (mx2 - x_p, my2 + 45), '%.1d' % (pixel[1]))

                    draw_str2(frame, (mx2 - x_p, my2 + 60), '%.1d' % (pixel[2]))

                else:

                    if mx2 < (W_View_size / 2):

                        x_p = -30

                    elif mx2 > (W_View_size / 2):

                        x_p = 60

                    else:

                        x_p = 30

                    draw_str2(frame, (mx2 - x_p, my2 - 60), '-HSV-')

                    draw_str2(frame, (mx2 - x_p, my2 - 45), '%.1d' % (pixel[0]))

                    draw_str2(frame, (mx2 - x_p, my2 - 30), '%.1d' % (pixel[1]))

                    draw_str2(frame, (mx2 - x_p, my2 - 15), '%.1d' % (pixel[2]))

            #----------------------------------------------

            

            cv2.imshow('mini CTS5 - Video', frame )

            cv2.imshow('mini CTS5 - Mask', mask)



        key = 0xFF & cv2.waitKey(1)

        

        if key == 27:  # ESC  Key

            break

        elif key == ord(' '):  # spacebar Key

            if View_select == 0:

                View_select = 1

            else:

                View_select = 0

        elif key == ord('s') or key == ord('S'):  # s or S Key:  Setting valus Save

            hsv_setting_save()

            msg_one_view = 1



    # cleanup the camera and close any open windows

    receiving_exit = 0

    time.sleep(0.5)

    

    camera.release()

    cv2.destroyAllWindows()





















