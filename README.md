# Patrasche-Project
### Patrasche-Project 
//(주)비타소프트

> # 파트라슈 프로젝트 Zed2 카메라, YDLidar 데이터수집 센서 개발
> > ### 담당자 강대현
# main.py   
## Zed2 카메라 센서와 YDLidar으로 Left, Right, Depth  거리, 기울기, Lidar 데이터를 저장하는 코드입니다   
### 저장 / 저장종료 : SpaceBar
### 프로그램 종료 : ESC   
```python
> # 저장경로
>> Left : left_{str(count).zfill(4)}.jpg # Zed 센서로 Left이미지를 저장합니다
>> Right : right_{str(count).zfill(4)}.jpg # Zed 센서로 Right이미지를 저장합니다
>> Depth : depth_{str(count).zfill(4)}.jpg # Zed 센서로 Depth이미지를 저장힙니다
>> Distance : distance_{str(count).zfill(4)}.npy # Zed 센서로 Distance 거리 데이터를 저장합니다 (정규화 되어있는 데이터)
>> Slope : slope_{str(count).zfill(4)}.npy # Zed 센서로 기울기를 저장합니다
>> degree : degree_{str(count).zfill(4)}.csv # Lidar 센서로 라이더 데이터를 저장합니다 (360도 전부 저장)
```
# Left, Right, Depth   
jpg 형식으로 저장되어 있습니다.      
프레임별로 카운트하여 zfill(4)으로 Left_0000.jpg 형식으로 저장됩니다

# Distance   
npy 형식으로 저장되어 있습니다.  
프레임별로 카운트하여 zfill(4)으로 distance_0000.jpg 형식으로 저장됩니다   
넘파이어레이 2차원리스트로 저장되어 있으며 0 ~ 255의 정규화 되어있습니다.

# Slope   
npy 형식으로 저장되어 있습니다.  
프레임별로 카운트하여 zfill(4)으로 slope_0000.jpg 형식으로 저장됩니다   
넘파이어레이 1차원리스트로 저장되어 있으며 [가로축, 세로축, 높이 ,대각선] 입니다

# degree   
csv 형식으로 저장되어 있습니다. 
프레임별로 카운트하여 zfill(4)으로 degree_0000.csv 형식으로 저장됩니다   
csv로 저장되어 있으며 한줄마다 "각도, 거리, 강도" 입니다.



# Ubuntu 18.04 환경설정 , GTX1650버전   


# nvidia 드라이버

## 안전부팅 설정 
https://m.blog.naver.com/PostView.naver?isHttpsRedirect=true&blogId=kmsoft&logNo=220785170138

엔비디아 470 드라이버 설치
```bash
#repository 추가 및 업데이트
sudo add-apt-repository ppa:graphics-drivers/ppa
sudo apt update
#원하는 버전의 nvidia-driver 수동 설치
sudo apt-get install nvidia-driver-470
#재부팅
sudo reboot
#확인
nvidia-smi
```
설치중 의존성 오류가 뜰경우
https://nirsa.tistory.com/330

# Cuda 11.2
https://settembre.tistory.com/449?category=948659

# ZED SDK 11.5
https://www.stereolabs.com/docs/installation/linux/


# YDLidar
sudo apt install git
sudo apt install cmake pkg-config

sudo apt-get install swig
sudo apt-get install python swig
sudo apt install python-pip
sudo apt-get install python-pip

# Cmake 의존 라이브러리 설치 (make , gcc, gcc-c++ openssl, openssl-devel)
sudo apt-get update
sudo apt-get install make
sudo apt-get install gcc 
sudo apt-get install g++
sudo apt-get install libssl-dev openssl
sudo apt-get install libssl-dev


# Cmake 설치
sudo wget https://cmake.org/files/v3.15/cmake-3.15.2.tar.gz
sudo tar xvfz cmake-3.15.2.tar.gz
cd cmake-3.15.2
sudo ./bootstrap
sudo make
sudo make install

# YDLidar-SDK설치
sudo git clone https://github.com/YDLIDAR/YDLidar-SDK.git
sudo mkdir YDLidar-SDK/build
cd YDLidar-SDK/build
sudo cmake ..
sudo make
sudo make install

cd YDLidar-SDK
sudo pip3 install .
sudo python3 setup.py build
sudo python3 setup.py install

# dev/ttyUSB0 과 프로젝트 전부에게 권한을 부여해야한다

# sh 더블클릭 실행 하는법
https://skylit.tistory.com/163
