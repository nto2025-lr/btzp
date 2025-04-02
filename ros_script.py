import rospy
import cv2
import sys
import os
import math
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String
from geometry_msgs.msg import PointStamped, Point
from cv_bridge import CvBridge
from clover import long_callback, srv
from std_srvs.srv import Trigger
import tf2_ros
import numpy as np
import tf2_geometry_msgs
import image_geometry
from pymavlink import mavutil
from mavros_msgs.srv import CommandLong
from mavros_msgs.msg import State
import pickle

rospy.init_node('cv', disable_signals=True)

get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
set_attitude = rospy.ServiceProxy('set_attitude', srv.SetAttitude)
land = rospy.ServiceProxy('land', Trigger)
send_command = rospy.ServiceProxy('mavros/cmd/command', CommandLong)

tf_buffer = tf2_ros.Buffer()
tf_listener = tf2_ros.TransformListener(tf_buffer)
camera_model = image_geometry.PinholeCameraModel()
camera_model.fromCameraInfo(rospy.wait_for_message('main_camera/camera_info', CameraInfo))


bridge = CvBridge()
color_text = {'red': '\033[31m\033[1m', 'green': '\033[32m\033[1m', 'yellow': '\033[33m\033[1m', 'cyan': '\033[36m\033[1m', 'white': '\033[37m\033[1m', 'blue': '\033[34m\033[1m'} #вот это короче вообще мага фича, рекомендую, позволяет текст цветной выводить
colors = {'red': (0, 0, 255), 'green': (0, 255, 0), 'yellow': (0, 255, 255), 'blue': (255, 0, 0)}
model_name = {'red': 'Найдено административное здание', 'green': 'Найдена лаборатория', 'yellow': 'Найден вход в шахту', 'blue': 'Найдено здание для обогащения угля'}
count_models = 1
length = 86.7
drone_state = {'preflight_check': 'ERROR', 'bat_V': 0, 'bat_proc': 0}
coord_red = []

try:
    img_map = cv2.imread('static/image.png')
except FileNotFoundError:
    pass

def check(x, y):
    for model in coord_red:
        x0, y0 = model
        if (x0 - x) ** 2 + (y0 - y) ** 2 <= 0.5:
            return False
    return True

def real_round(number: float, poz=0):
    a = number * 10 * 10 ** poz
    return math.floor(a / 10) / 10 ** poz if a % 10 < 5 else math.ceil(a / 10) / 10 ** poz

def land_wait():
    land()
    while get_telemetry().armed:
        rospy.sleep(0.2)
    print(f'{color_text["yellow"]}[DISARM]{color_text["white"]}')

def navigate_wait(x=0, y=0, z=0, yaw=float('nan'), speed=0.6, frame_id='aruco_map', auto_arm=False, tolerance=0.2):
    navigate(x=x, y=y, z=z, yaw=yaw, speed=speed, frame_id=frame_id, auto_arm=auto_arm)
    if auto_arm:
        print(f'{color_text["yellow"]}[ARM]{color_text["white"]}')
        print(f'Takeoff by {color_text["green"]}Body{color_text["white"]} Z = {float(z)}')
    else:
        print(f'Flight to {color_text["cyan"]}{frame_id.capitalize()}{color_text["white"]}')

    while not rospy.is_shutdown():
        try:
            telem = get_telemetry(frame_id='navigate_target')
        except rospy.ServiceException:
            continue

        if math.sqrt(telem.x ** 2 + telem.y ** 2 + telem.z ** 2) < tolerance:
            break
        rospy.sleep(0.2)

def img_xy_to_point(xy, dist):
    xy_rect = camera_model.rectifyPoint(xy)
    ray = camera_model.projectPixelTo3dRay(xy_rect)
    return Point(x=ray[0] * dist, y=ray[1] * dist, z=dist)

def get_center_of_mass(mask):
    M = cv2.moments(mask)
    if M['m00'] == 0:
        return None
    return M['m10'] // M['m00'], M['m01'] // M['m00']

def color_coord(mask=math.nan, msg=math.nan):
    global count_models, coord_red
    xy = get_center_of_mass(mask)
    try:
        altitude = get_telemetry('terrain').z
    except rospy.ServiceException:
        return
    xy3d = img_xy_to_point(xy, altitude)
    # print(xy3d)
    target = PointStamped(header=msg.header, point=xy3d)
    try:
        setpoint = tf_buffer.transform(target, 'aruco_map', timeout=rospy.Duration(0.2))
    except rospy.ServiceException as e:
        rospy.logerr('Transform error: {}'.format(e))
    if check(setpoint.point.x, setpoint.point.y):
        '''with open('static/otchet.txt', 'a') as file:
            file.write(f'{model_name[color_name]}:\n X = {real_round(setpoint.point.x, 2)} Y = {real_round(setpoint.point.y, 2)} color = {color_name}\n')
            file.write('\n')'''
        
        print(f'X = {real_round(setpoint.point.x, 2)} Y = {real_round(setpoint.point.y, 2)}')
        coord_red.append((setpoint.point.x, setpoint.point.y))
        
        '''with open('static/coord_model.pkl', 'wb') as file_pkl:
            pickle.dump(opr_models, file_pkl)
        x, y, color = tuple(opr_models[count_models])
        bgr_color = colors[color]
        cv2.rectangle(img_map, (108 + int(real_round((x - 0.3) * length)), 892 - int(real_round((y - 0.3) * length))), (112 + int(real_round((x + 0.3) * length)), 888 - int(real_round((y + 0.3) * length))), (0, 0, 0), -1)
        cv2.rectangle(img_map, (110 + int(real_round((x - 0.3) * length)), 890 - int(real_round((y - 0.3) * length))), (110 + int(real_round((x + 0.3) * length)), 890 - int(real_round((y + 0.3) * length))), bgr_color, -1)
        cv2.putText(img_map, f'color = {color_name}', (90 + int(real_round((x - 0.3) * length)), 905 - int(real_round((y - 0.3) * length))), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 5)
        cv2.putText(img_map, f'color = {color_name}', (90 + int(real_round((x - 0.3) * length)), 905 - int(real_round((y - 0.3) * length))), cv2.FONT_HERSHEY_SIMPLEX, 0.5, bgr_color, 2)
        cv2.putText(img_map, f'X = {real_round(setpoint.point.x, 2)}', (100 + int(real_round((x - 0.3) * length)), 920 - int(real_round((y - 0.3) * length))), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 5)
        cv2.putText(img_map, f'X = {real_round(setpoint.point.x, 2)}', (100 + int(real_round((x - 0.3) * length)), 920 - int(real_round((y - 0.3) * length))), cv2.FONT_HERSHEY_SIMPLEX, 0.5, bgr_color, 2)
        cv2.putText(img_map, f'Y = {real_round(setpoint.point.y, 2)}', (100 + int(real_round((x - 0.3) * length)), 935 - int(real_round((y - 0.3) * length))), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 5)
        cv2.putText(img_map, f'Y = {real_round(setpoint.point.y, 2)}', (100 + int(real_round((x - 0.3) * length)), 935 - int(real_round((y - 0.3) * length))), cv2.FONT_HERSHEY_SIMPLEX, 0.5, bgr_color, 2)

        cv2.imwrite('static/image.png', img_map)'''
        count_models += 1

shutdown_initiated = True

@long_callback
def image_callback(msg):
    global shutdown_initiated
    if shutdown_initiated:
        return
    # img = bridge.imgmsg_to_cv2(msg, 'bgr8')
    # hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    # lower_red1 = np.array([0, 100, 100])     # Первый диапазон (0-10)
    # upper_red1 = np.array([15, 255, 255])
    # lower_red2 = np.array([0, 100, 100])  # Второй диапазон (170-180)
    # upper_red2 = np.array([15, 255, 255])

    # # Создание масок и их объединение
    # mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
    # mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
    # red = cv2.bitwise_or(mask1, mask2)

    # kernel = np.ones((5, 5), np.uint8)
    # red = cv2.morphologyEx(red, cv2.MORPH_CLOSE, kernel)  # Удаление шума
    # red = cv2.morphologyEx(red, cv2.MORPH_OPEN, kernel)
    
    # num_labels, labels = cv2.connectedComponents(red)
    # for i in range(1, num_labels):
    #     component_mask = np.where(labels == i, 255, 0).astype(np.uint8)
    #     if cv2.countNonZero(component_mask) > 200:
    #         color_coord(mask=component_mask, msg=msg)
    #     else:
    #         continue

    rospy.sleep(0.3)

sub = rospy.Subscriber('main_camera/image_raw', Image, image_callback, queue_size=1)

def main():
    global shutdown_initiated, drone_state
    img_map = cv2.resize(bridge.imgmsg_to_cv2(rospy.wait_for_message('aruco_map/image', Image), 'bgr8'), (1000, 1000))
    cv2.imwrite('static/image.png', img_map)
    # bat_V = rospy.wait_for_message('/mavros/battery', String)
    # pr_ch = rospy.wait_for_message('/mavros/state', String)
    # print(bat_V)
    # print(pr_ch)
    navigate_wait(z=1, frame_id='body', auto_arm=True, speed=0.5)
    shutdown_initiated = False
    navigate_wait(z=1, x=0.5, y=3, frame_id='aruco_map', speed=0.6)
    navigate_wait(z=1, x=3.5, y=0.5, frame_id='aruco_map', speed=0.3)
    shutdown_initiated = True
    rospy.sleep(3)
    # for stage, aruco_id in enumerate([9, 19, 10, 20, 29, 39, 30, 40, 49, 59, 50, 60, 69, 79, 70, 80, 89, 99, 90][stage_nav:], start=stage_nav):
    #     navigate_wait(z=1.5, frame_id=f'aruco_{aruco_id}')
    #     with open('static/stage.pkl', 'wb') as file:
    #         pickle.dump(stage + 1, file)
    navigate_wait(z=1.5, x=0.5, frame_id='aruco_152', speed=0.7, tolerance=0.3, yaw=float(0))
    navigate_wait(z=0.2, x=0.5, frame_id='aruco_152', speed=0.3, tolerance=0.15, yaw=float(0))
    land_wait()
    sub.unregister()
    shutdown_initiated = True
    rospy.sleep(0.3) 
    exit()
if len(sys.argv) == 1:
    main()
    rospy.spin()

elif len(sys.argv) == 2 and sys.argv[1]== '--land':
    land_wait()
    print(f'{color_text["red"]}[LAND]{color_text["white"]}')
    exit()
elif len(sys.argv) == 2 and sys.argv[1]== '--home':
    navigate_wait(z=1.5, x=0.5, frame_id='aruco_152', speed=0.7, tolerance=0.3, yaw=float(0))
    navigate_wait(z=0.2, x=0.5, frame_id='aruco_152', speed=0.3, tolerance=0.15, yaw=float(0))
    land_wait()
    print(f'{color_text["red"]}[LAND]{color_text["white"]}')
    exit()
elif len(sys.argv) == 2 and sys.argv[1]== '--stop':
    navigate_wait(x=0, y=0, frame_id='body', speed=0.6, tolerance=0.2, yaw=float(0))
    exit()
elif len(sys.argv) == 2 and sys.argv[1]== '--kill':
    # send_command(command=mavutil.mavlink.MAV_CMD_DO_FLIGHTTERMINATION, param1=1)
    if get_telemetry().armed:
        set_attitude(thrust=0)
    else:
        print(f'{color_text["red"]}Дрон не заармлен{color_text["white"]}')
    print(f'{color_text["red"]}[KILL]{color_text["white"]}')
    exit()
# print(len(sys.argv))
# python3 new_ros.py --stop
# python3 new_ros.py --land
# python3 new_ros.py --kill
# python3 new_ros.py --home
