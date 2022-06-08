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

# ZED SETTING
zed = sl.Camera()
runtime = sl.InitParameters()  # 객체 생성
runtime.camera_resolution = sl.RESOLUTION.HD2K
runtime.camera_fps = 15  # Set fps at 60
runtime.coordinate_units = sl.UNIT.METER
runtime.depth_minimum_distance = 0.15
runtime.depth_maximum_distance = 40
runtime.depth_mode = sl.DEPTH_MODE.ULTRA  # depth 해상도 ULTRA
sensors_data = sl.SensorsData()

# YDLIDAR SETTING
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

# PROGRAM SETTING
f = open("path.txt", 'r', encoding='UTF8' )
line = f.readline()

que = Queue()
status = False  # 키 입력 확인
save_path = str(line)  # 저장 파일명
root_path = '' # 저장 경로
frm_path = ''  # 저장 프레임
count = 0  # 저장 카운트



def getdegree(angle): # 원주를 각도로 변환하는 함수 
    degree = math.degrees(angle)
    if degree < 0:
        degree = 360 + degree
    degree = int(degree)
    return degree

def DepthNormalizing(data): # 데이터 정규화하는 함수
    max_data, min_data = np.max(data), np.min(data)
    data = (data - min_data) / (max_data - min_data) * 255
    return data


def distance_undefined(nd_array): # inf -inf데이터 0으로 변환하는 함수
    nd_array[nd_array == inf] = 0
    nd_array[nd_array == -inf] = 0
    np.nan_to_num(nd_array)
    return nd_array


def file_writer(): # 큐 자료구조를 읽어서 데이터 저장하는 함수 (쓰레드)
    global que, root_path
    while True:
        if not que.empty():
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


def record(): # 큐 자료구조에 데이터를 저장하는 함수
    global que, count, zed, runtime, status
    
    # ZED
    left = sl.Mat()
    right = sl.Mat()
    depth = sl.Mat()
    dis = sl.Mat()
    
    # LIDAR
    ret = laser.initialize()
    if ret:
        ret = laser.turnOn()
        scan = ydlidar.LaserScan()
    while ret and ydlidar.os_isOk():
        if status:
            if zed.grab(runtime) == sl.ERROR_CODE.SUCCESS:
                ret = laser.turnOn()
                scan = ydlidar.LaserScan()
                zed.retrieve_image(left, sl.VIEW.LEFT)  # images
                zed.retrieve_image(right, sl.VIEW.RIGHT)
                zed.retrieve_image(depth, sl.VIEW.DEPTH)
                zed.retrieve_measure(dis, sl.MEASURE.DEPTH)  # distance
                path = os.path.join(save_path, frm_path)
                zed.get_sensors_data(sensors_data, sl.TIME_REFERENCE.CURRENT)
                quaternion = sensors_data.get_imu_data().get_pose().get_orientation().get()
                r = laser.doProcessSimple(scan)
                if r:
                    ang = []
                    last_degree = 0
                    for point in scan.points:
                        degree = getdegree(point.angle)
                        if degree != last_degree:
                            # print(degree, last_degree)
                            ang.append([degree, point.range, point.intensity])
                            last_degree = degree
                        else:
                            pass
                            # print('Failed to get Lidar Data')
                ang.sort(key=lambda x:x[0])
                

                # Save image
                # 큐 저장형식 {딕셔너리이름  : 파일경로, 데이터}
                que.put({'left': [os.path.join(path, f'left_{str(count).zfill(4)}.jpg'), left.get_data()]})
                que.put({'right': [os.path.join(path, f'right_{str(count).zfill(4)}.jpg'), right.get_data()]})
                que.put({'depth': [os.path.join(path, f'depth_{str(count).zfill(4)}.jpg'), depth.get_data()]})
                que.put({'dis': [os.path.join(path, f'distance_{str(count).zfill(4)}.npy'),
                                 DepthNormalizing(distance_undefined(dis.get_data()))]})
                que.put({'slope': [os.path.join(path, f'slope_{str(count).zfill(4)}.npy'), quaternion]})
                que.put({'degree': [os.path.join(path, f'degree_{str(count).zfill(4)}.csv'), ang]})
                count += 1
        else:
            count = 0


def main(): # 메인화면, 센서 연결확인
    global count, zed, runtime, status, frm_path
    err = zed.open(runtime)
    if err != sl.ERROR_CODE.SUCCESS:
        print('제드 카메라가 연결되어있지 않던가 이미 카메라가 켜져있습니다')
        exit()
    else:
        print('제드 카메라 연결성공!')
        
    # 제드카메라 환경설정
    runtime = sl.RuntimeParameters()
    runtime.sensing_mode = sl.SENSING_MODE.FILL
    left = sl.Mat()
    
    # 크기 가져오기
    image_size = zed.get_camera_information().camera_resolution
    # 크기 리사이징
    image_size.width = image_size.width / 2.5
    image_size.height = image_size.height / 2.5

    print("\n스페이스바: 촬영을 시작/종료\nESC : 프로그램종료\n")
    while True:

        if zed.grab(runtime) == sl.ERROR_CODE.SUCCESS:

            # 화면출력
            zed.retrieve_image(left, sl.VIEW.LEFT)
            resized_left = cv2.resize(left.get_data(), (image_size.width, image_size.height))
            cv2.putText(resized_left, str(count), (10, 100), cv2.FONT_HERSHEY_SIMPLEX, 3, (0, 0, 255), 3)
            cv2.imshow('ZED_left', resized_left)
        else:
            cv2.destroyAllWindows()
            zed.close()
            break
        
        key = cv2.waitKey(5)  # Ese 누르면 프로그램 종료
        if key == 27:
            break
        elif key == 32:  # Space bar 누르면 record 시작
            status = not status
            if status:
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
                count = 0
                frm_path = ''
                root_path = ''
    cv2.destroyAllWindows()
    zed.close()
    laser.turnOff();
    laser.disconnecting();
    sys.exit(0)
    

if __name__ == "__main__":
    que_thread = threading.Thread(target=file_writer, args=(), daemon=True)
    que_thread.start()
    record_Thread = threading.Thread(target=record, args=(), daemon=True)
    record_Thread.start()
    main()
    record_Thread.join()
    que_thread.join()
