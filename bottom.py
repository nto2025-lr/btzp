# подключение дополнительных модулей python
import rospy
import sys
import math
from clover import srv
from std_srvs.srv import Trigger
rospy.init_node('cv')
# подключение функций ros
get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
land = rospy.ServiceProxy('land', Trigger)
# функция полета с ожиданием
def navigate_wait(x=0, y=0, z=0, yaw=float('nan'), speed=0.6, frame_id='aruco_map', auto_arm=False, tolerance=0.2):
    navigate(x=x, y=y, z=z, yaw=yaw, speed=speed, frame_id=frame_id, auto_arm=auto_arm)
    if auto_arm:
        print(f'[ARM]')

    while not rospy.is_shutdown():
        try:
            telem = get_telemetry(frame_id='navigate_target')
        except rospy.ServiceException:
            continue

        if math.sqrt(telem.x ** 2 + telem.y ** 2 + telem.z ** 2) < tolerance:
            break
        rospy.sleep(0.2)

def land_wait():  # функция писадки с ожиданием
    land()
    while get_telemetry().armed:
        rospy.sleep(0.2)
    
if len(sys.argv) == 2 and sys.argv[1]== '--land': 
    land_wait()
    print('[LAND]')
    exit()
elif len(sys.argv) == 2 and sys.argv[1]== '--home':
    navigate_wait(z=1, x=0.5, frame_id='aruco_152', speed=0.7, tolerance=0.3, yaw=float(0))
    navigate_wait(z=0.2, x=0.5, frame_id='aruco_152', speed=0.3, tolerance=0.15, yaw=float(0))
    land_wait()
    print('[LAND]')
    exit()
elif len(sys.argv) == 2 and sys.argv[1]== '--stop':
    telemm = get_telemetry(frame_id='aruco_map')
    x0, y0 = telemm.x, telemm.y
    navigate_wait(x=x0, y=y0, z=1, frame_id='aruco_map', speed=0.6, tolerance=0.2, yaw=float(0))
    exit()
elif len(sys.argv) == 2 and sys.argv[1]== '--kill':
    # if get_telemetry().armed:
    #     # arming(False)
    #     send_command(command=mavutil.mavlink.MAV_CMD_DO_FLIGHTTERMINATION, param1=1)

    # else:
    #     print(f'{color_text["red"]}Дрон не заармлен{color_text["white"]}')
    # print(f'{color_text["red"]}[KILL]{color_text["white"]}')
    exit()
