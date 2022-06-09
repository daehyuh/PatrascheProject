import os
import pyzed.sl as sl
import ydlidar
import cv2
import numpy as np
import threading
import sys
import time
import math
import csv
from queue import Queue
from numpy import inf
from datetime import datetime

# ===== ZED SETTING =====
zed = sl.Camera()
runtime = sl.InitParameters()  # 객체 생성
runtime.camera_resolution = sl.RESOLUTION.HD2K # 2K 해상도
runtime.camera_fps = 15  # Set fps at 60
runtime.coordinate_units = sl.UNIT.METER
runtime.depth_minimum_distance = 0.15 # 최소 거리
runtime.depth_maximum_distance = 40 # 최대 거리
runtime.depth_mode = sl.DEPTH_MODE.ULTRA  # depth 해상도 ULTRA
sensors_data = sl.SensorsData()


# ==== YDLIDAR SETTING =====
ydlidar.os_init();
ports = ydlidar.lidarPortList();
port = "/dev/ttyUSB0";
for key, value in ports.items():
    port = value;
laser = ydlidar.CYdLidar();

laser.setlidaropt(ydlidar.LidarPropSerialPort, port);
laser.setlidaropt(ydlidar.LidarPropSerialBaudrate, 230400);
laser.setlidaropt(ydlidar.LidarPropLidarType, ydlidar.TYPE_TRIANGLE);
laser.setlidaropt(ydlidar.LidarPropDeviceType, ydlidar.YDLIDAR_TYPE_SERIAL);
laser.setlidaropt(ydlidar.LidarPropScanFrequency, 12.0);
laser.setlidaropt(ydlidar.LidarPropSampleRate, 5);
laser.setlidaropt(ydlidar.LidarPropSingleChannel, False);
laser.setlidaropt(ydlidar.LidarPropMaxAngle, 180.0);
laser.setlidaropt(ydlidar.LidarPropMinAngle, -180.0);
laser.setlidaropt(ydlidar.LidarPropMaxRange, 16.0);
laser.setlidaropt(ydlidar.LidarPropMinRange, 0.28);
laser.setlidaropt(ydlidar.LidarPropIntenstiy, True);
# LIDAR
ret = laser.initialize()
if ret:
    ret = laser.turnOn()
    scan = ydlidar.LaserScan()

# ==== PROGRAM SETTING ====
que = Queue()
f = open("path.txt", 'r', encoding='UTF8' )
save_path = f.readline() # 저장 폴더명
root_path = '' # 저장 경로
status = False  # 키 입력 확인
frm_path = ''  # 저장 프레임
count = 0  # 저장 카운트


# ==== FUNCTION ====
def getDegree(angle): # 원주를 각도로 변환하는 함수 
    degree = math.degrees(angle)
    if degree < 0:
        degree = 360 + degree
    degree = int(degree)
    return degree

def depthNormalizing(data): # 데이터 정규화하는 함수
    max_data, min_data = np.max(data), np.min(data)
    data = (data - min_data) / (max_data - min_data) * 255
    return data


def distance_Undefined(nd_array): # inf -inf데이터 0으로 변환하는 함수
    nd_array[nd_array == inf] = 0
    nd_array[nd_array == -inf] = 0
    np.nan_to_num(nd_array)
    return nd_array


def file_Writer(): # 큐에 쌓인 데이터를 파일로 저장하는 함수 (쓰레드)
    global que, root_path
    while True:
        if not que.empty():
            # print(que.queue)
            que_dict = que.get()
            for key, value in que_dict.items():
                if key == "dis" or key == "slope":
                    np.save(value[0], value[1])
                elif key == "degree":
                    with open(value[0],'w', newline='') as f:
                        wr = csv.writer(f)
                        wr.writerows(value[1])
                        f.close()
                else:
                    cv2.imwrite(value[0], value[1])
                print(value[0])

def record(left,right,depth,dis,quaternion,ang): # 큐에 데이터를 저장하는 함수
    global que, count, zed, runtime, status, root_path                
    
    # 큐 저장형식 {딕셔너리이름  : 파일경로, 데이터}
    que.put({'left': [os.path.join(root_path, f'left_{str(count).zfill(4)}.jpg'), left.get_data()]})
    que.put({'right': [os.path.join(root_path, f'right_{str(count).zfill(4)}.jpg'), right.get_data()]})
    que.put({'depth': [os.path.join(root_path, f'depth_{str(count).zfill(4)}.jpg'), depth.get_data()]})
    que.put({'dis': [os.path.join(root_path, f'distance_{str(count).zfill(4)}.npy'),
                     depthNormalizing(distance_Undefined(dis.get_data()))]})
    que.put({'slope': [os.path.join(root_path, f'slope_{str(count).zfill(4)}.npy'), quaternion]})
    que.put({'degree': [os.path.join(root_path, f'degree_{str(count).zfill(4)}.csv'), ang]})
    count += 1

def main_Stop(): # 프로그램을 종료시키는 함수 (저장할 데이터 있는지 확인)
    while(True):
        if (que.empty()):
            sys.exit()
            
def main(): # 메인화면, 센서 연결확인
    global count, zed, runtime, status, frm_path, root_path
    err = zed.open(runtime)
    if err != sl.ERROR_CODE.SUCCESS:
        print('제드 카메라가 연결되어있지 않던가 이미 카메라가 켜져있습니다')
        cv2.destroyAllWindows()
        zed.close()
        laser.turnOff();
        laser.disconnecting();
        main_stop()
    else:
        print('제드 카메라 연결성공!')
        
    # ZED
    runtime = sl.RuntimeParameters()
    runtime.sensing_mode = sl.SENSING_MODE.FILL
    left = sl.Mat()
    right = sl.Mat()
    depth = sl.Mat()
    dis = sl.Mat()
    
    # 크기 리사이징
    image_size = zed.get_camera_information().camera_resolution
    image_size.width = image_size.width / 2.5
    image_size.height = image_size.height / 2.5
    
    print("\n스페이스바: 촬영을 시작/종료\nESC : 프로그램종료\n")
    while True:
        # =============화면 출력=============
        if zed.grab(runtime) == sl.ERROR_CODE.SUCCESS and (ret and ydlidar.os_isOk()):
            # ZED 데이터 가져오기
            zed.retrieve_image(left, sl.VIEW.LEFT)  # images
            
            # Left 화면 출력하기
            resized_left = cv2.resize(left.get_data(), (image_size.width, image_size.height))
            cv2.putText(resized_left, str(count), (10, 100), cv2.FONT_HERSHEY_SIMPLEX, 3, (0, 0, 255), 3)
            cv2.imshow('ZED_left', resized_left)
        else:
            cv2.destroyAllWindows()
            zed.close()
            break
            
        # =============데이터 저장 (status) =============
        if status: # 저장 status 확인
            zed.retrieve_image(left, sl.VIEW.LEFT)
            zed.retrieve_image(right, sl.VIEW.RIGHT)
            zed.retrieve_image(depth, sl.VIEW.DEPTH)
            zed.retrieve_measure(dis, sl.MEASURE.DEPTH)  # distance
            zed.get_sensors_data(sensors_data, sl.TIME_REFERENCE.CURRENT)
            quaternion = sensors_data.get_imu_data().get_pose().get_orientation().get()
            r = laser.doProcessSimple(scan)
            if r:
                ang = []
                last_degree = 0
                for point in scan.points:
                    degree = getDegree(point.angle)
                    if degree != last_degree:
                        # print(degree, last_degree)
                        ang.append([degree, point.range, point.intensity])
                        last_degree = degree
                    else:
                        pass
                        # print('Failed to get Lidar Data')
            ang.sort(key=lambda x:x[0])

            record(left,right,depth,dis,quaternion,ang) # 저장
        
        # =============키보드 이벤트 (ESC, 스페이스바, status) =============
        key = cv2.waitKey(5)  # Ese 누르면 프로그램 종료
        if key == 27:
            break
        elif key == 32:  # Space bar 누르면 record 시작
            status = not status
            if status:
                count = 0
                frm_path = ''
                root_path = ''
                frm_path = datetime.today().strftime('%Y%m%d_%H%M%S%f')
                root_path = os.path.join(save_path, frm_path)
                os.makedirs(root_path, exist_ok=True)  # 데이터 저장 경로
                # os.makedirs(root_path + "/video", exist_ok=True)  # 비디오 변환 저장
                print(f"녹화시작 {root_path}에 파일이 저장됩니다.")
            else:
                # 비디오 변환
                # os.system(
                #     "ffmpeg -f image2 -r 5 -i " + root_path + "/depth_%04d.jpg -vcodec mpeg4 -y " + root_path + "/video/depth_images_convert_video.mp4")
                print(f"{count}개의 파일이 {root_path}에 저장되었습니다.")
                print("\n스페이스바를 눌러 촬영을 시작/종료 하세요\nESC : 프로그램종료\n")
    cv2.destroyAllWindows()
    zed.close()
    laser.turnOff();
    laser.disconnecting();
    main_Stop()
    
    
if __name__ == "__main__":
    que_thread = threading.Thread(target=file_Writer, args=(), daemon=True)
    que_thread.start()
    main()
    que_thread.join()