# 과학기술정보통신부 정보통신산업진흥원 AI 바우처 지원사업
## 데이터 수집 및 전처리, 라벨링 작업
## Patrasche-Project 
Traffic Object Detection, Drivable Area Segmentation, Lane Detection을 위한 멀티모달 학습 데이터 구축


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
# 설치중 의존성 오류가 뜰경우
[링크](https://nirsa.tistory.com/330)

# Cuda 11.2
https://settembre.tistory.com/449?category=948659

# ZED SDK 11.5
[링크](https://www.stereolabs.com/docs/installation/linux/)

# YDLidar
```
sudo apt install git
sudo apt install cmake pkg-config

sudo apt-get install swig
sudo apt-get install python swig
sudo apt install python-pip
sudo apt-get install python-pip
```
# Cmake 의존 라이브러리 설치 (make , gcc, gcc-c++ openssl, openssl-devel)
```
sudo apt-get update
sudo apt-get install make
sudo apt-get install gcc 
sudo apt-get install g++
sudo apt-get install libssl-dev openssl
sudo apt-get install libssl-dev
```
# Cmake 설치
```
sudo wget https://cmake.org/files/v3.15/cmake-3.15.2.tar.gz
sudo tar xvfz cmake-3.15.2.tar.gz
cd cmake-3.15.2
sudo ./bootstrap
sudo make
sudo make install
```
# YDLidar-SDK설치
```
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
```
# dev/ttyUSB0 과 프로젝트 전부에게 권한을 부여해야한다

# sh 더블클릭 실행 하는법
[링크](https://skylit.tistory.com/163)
