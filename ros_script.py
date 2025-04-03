import rospy
import cv2
import sys
import requests
import math
from mavros_msgs.srv import CommandBool
from sensor_msgs.msg import Image, CameraInfo, BatteryState
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
from sensor_msgs.msg import BatteryState
rospy.init_node('cv', disable_signals=True)

get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
set_attitude = rospy.ServiceProxy('set_attitude', srv.SetAttitude)
land = rospy.ServiceProxy('land', Trigger)
send_command = rospy.ServiceProxy('mavros/cmd/command', CommandLong)
jet_pub = rospy.Publisher('jet', Image, queue_size=1)
tf_buffer = tf2_ros.Buffer()
arming = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
tf_listener = tf2_ros.TransformListener(tf_buffer)
camera_model = image_geometry.PinholeCameraModel()
camera_model.fromCameraInfo(rospy.wait_for_message('main_camera/camera_info', CameraInfo))


bridge = CvBridge()
color_text = {'red': '\033[31m\033[1m', 'green': '\033[32m\033[1m', 'yellow': '\033[33m\033[1m', 'cyan': '\033[36m\033[1m', 'white': '\033[37m\033[1m', 'blue': '\033[34m\033[1m'} #вот это короче вообще мага фича, рекомендую, позволяет текст цветной выводить
video = False
line_teplo_state = False

try:
    img_map = cv2.imread('static/image.png')
except FileNotFoundError:
    pass

def aruco_map_to_image(x, y):
    left_up = (278, 113)
    d_xy = 111
    y=7-y
    x_img = int(left_up[0]+x*d_xy)
    y_img = int(left_up[1]+y*d_xy)
    return (x_img, y_img)
def convert_aruco(x, y):
    x = y
    y = 7 - x
    return (x, y)

def line_temp(msg, gray):
    global line_teplo_state
    file = open("static/otchet.txt", "w")
    file.write("Список участков \n")

    _, binary = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY)

    # Находим контуры
    contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    num_labels, labels = cv2.connectedComponents(binary)
    for i in range(1, num_labels):
        component_mask = np.where(labels == i, 255, 0).astype(np.uint8)
        if cv2.countNonZero(component_mask) > 200:
            contours, _ = cv2.findContours(component_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            largest_contour = max(contours, key=cv2.contourArea)
            # Находим две самые удаленные точки контура
            max_dist = 0
            start_point, end_point = None, None
            
            for i in range(len(largest_contour)):
                for j in range(i + 1, len(largest_contour)):
                    dist = np.linalg.norm(largest_contour[i][0] - largest_contour[j][0])
                    if dist > max_dist:
                        max_dist = dist
                        start_point = tuple(largest_contour[i][0])
                        end_point = tuple(largest_contour[j][0])

            start_point = tuple(map(int, start_point))
            end_point = tuple(map(int, end_point))
            print(f"Начало линии: {start_point}")
            print(f"Конец линии: {end_point}")
            start_point = color_coord(start_point, msg=msg)
            end_point = color_coord(start_point, msg=msg)
            string = f"{start_point} {end_point} \n"
            file.write(string)
            cv2.line(img_map, start_point, end_point, (0,0,255))
        else:
            continue
    cv2.imwrite('static/image.png', img_map)
    file.close()

def battery_callback(data):
    voltage = data.voltage
    curr = (real_round(voltage, 2)-14) // 0.028
    with open("static/info.txt", 'w+') as f:
        f.write(f"Preflight check:____Battery_V:____Battery_%:\n______OK____________{real_round(voltage, 2)}__________{int(curr)}")
        f.close()
    rospy.sleep(0.5)
    

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

def color_coord(xy=math.nan, msg=math.nan):
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
    return (real_round(setpoint.point.x, 2), real_round(setpoint.point.y, 2))

shutdown_initiated = True

@long_callback
def image_callback(msg):
    global shutdown_initiated, video, out, out2, cap, line_teplo_state
    if shutdown_initiated:
        return
    img = bridge.imgmsg_to_cv2(msg, 'bgr8')
    # hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    
    _, frame = cap.read()
    frame = cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)
    h, w, _ = frame.shape
    frame = frame[0:h, 0:w//2]
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    img_jet = cv2.applyColorMap(gray, 2)
    jet_pub.publish(bridge.cv2_to_imgmsg(img_jet, 'bgr8'))
    if video:
        out.write(img)
        out2.write(img_jet)
    if line_teplo_state:
        line_temp(msg, gray_teplo)
        line_teplo_state=False

    rospy.sleep(0.2)

sub = rospy.Subscriber('main_camera/image_raw', Image, image_callback, queue_size=1)
bat = rospy.Subscriber("/mavros/battery", BatteryState, battery_callback)
def main():
    global shutdown_initiated, video, out, out2, cap, gray_teplo, line_teplo_state
    width, width2 = 320, 256
    height, height2 = 240, 192
    fps = 5
    fourcc = cv2.VideoWriter_fourcc('m', 'p', '4', 'v')  # Кодек для MP4
    out = cv2.VideoWriter('output.mp4', fourcc, fps, (width, height))

    cap = cv2.VideoCapture(1)
    out2 = cv2.VideoWriter('output_JET.mp4', fourcc, fps, (width2, height2))
    _, frame0 = cap.read()
    frame0 = cv2.rotate(frame0, cv2.ROTATE_90_COUNTERCLOCKWISE)
    h, w, _ = frame0.shape
    frame0 = frame0[0:h, 0:w//2]
    gray_teplo = cv2.cvtColor(frame0, cv2.COLOR_BGR2GRAY)

    img_map = cv2.resize(bridge.imgmsg_to_cv2(rospy.wait_for_message('aruco_map/image', Image), 'bgr8'), (1000, 1000))
    cv2.imwrite('static/image.png', img_map)
    navigate_wait(z=2, frame_id='body', auto_arm=True, speed=0.5)
    navigate_wait(z=2, x=2, y=2.25, frame_id='aruco_map', speed=0.5)
    
    line_teplo_state=True
    navigate_wait(z=1.5, x=0.4, y=3.05, frame_id='aruco_map', speed=0.4)
    
    shutdown_initiated, video = False, True
    navigate_wait(z=1.5, x=3.4, y=0.55, frame_id='aruco_map', speed=0.3)
    shutdown_initiated, video= True, False
    out.release()
    out2.release()
    rospy.sleep(3)
    # response = requests.get("http://157.180.22.113:8080/coords_1.json")
    # server_data = response.json()
    server_data = {"nodes": [{"x": 1.0, "y": 6.5}, {"x": 3.55, "y": 4.45}], "tappings": [{"x": 1.57, "y": 6.04, "legal": 1}, {"x": 1.8, "y": 5.85, "legal": 0}, {"x": 2.20, "y": 5.54, "legal": 1}, {"x": 3.15, "y": 4.77, "legal": 0}]}
    navigate_wait(z=1.5, x=0, y=2, frame_id='aruco_map', speed=0.7, tolerance=0.3, yaw=float(0))
    navigate_wait(z=1.5, x=0, y=5, frame_id='aruco_map', speed=0.5, tolerance=0.3, yaw=float(0))

    navigate_wait(z=1.5, x=server_data['nodes'][0]['x'], y=server_data['nodes'][0]['y'], frame_id='aruco_map', speed=0.3, tolerance=0.3, yaw=float(0))
    count_img = 1
    for i in server_data['tappings']:
        x_t, y_t, leg = i['x'], i['y'], i['legal']
        if leg:
            navigate_wait(z=1.5, x=x_t, y=y_t, frame_id='aruco_map', speed=0.3, tolerance=0.3, yaw=float(0))
        else:
            img = bridge.imgmsg_to_cv2(rospy.wait_for_message('main_camera/image_raw', Image), 'bgr8')
            cv2.imwrite(f"image/image{count_img}.png", img)
        count_img += 1

    navigate_wait(z=1.5, x=server_data['nodes'][1]['x'], y=server_data['nodes'][1]['y'], frame_id='aruco_map', speed=0.3, tolerance=0.3, yaw=float(0))
    navigate_wait(z=1.5, x=0, y=5, frame_id='aruco_map', speed=0.7, tolerance=0.3, yaw=float(0))
    navigate_wait(z=1.5, x=0, y=2, frame_id='aruco_map', speed=0.7, tolerance=0.3, yaw=float(0))

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
    navigate_wait(z=1, x=0.5, frame_id='aruco_152', speed=0.7, tolerance=0.3, yaw=float(0))
    navigate_wait(z=0.2, x=0.5, frame_id='aruco_152', speed=0.3, tolerance=0.15, yaw=float(0))
    land_wait()
    print(f'{color_text["red"]}[LAND]{color_text["white"]}')
    exit()
elif len(sys.argv) == 2 and sys.argv[1]== '--stop':
    telemm = get_telemetry(frame_id='aruco_map')
    x0, y0 = telemm.x, telemm.y
    navigate_wait(x=x0, y=y0, z=1, frame_id='aruco_map', speed=0.6, tolerance=0.2, yaw=float(0))
    exit()
elif len(sys.argv) == 2 and sys.argv[1]== '--kill':
    # send_command(command=mavutil.mavlink.MAV_CMD_DO_FLIGHTTERMINATION, param1=1)
    if get_telemetry().armed:
        arming(False)
    else:
        print(f'{color_text["red"]}Дрон не заармлен{color_text["white"]}')
    print(f'{color_text["red"]}[KILL]{color_text["white"]}')
    exit()
rospy.spin()
