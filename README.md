# iAHRS ROS Package

===[Device Setting]====================================

-iAHRS는 USB로 연결되는데 상황에 따라서 인식되는 ttyUSB 포트의 번호가 바뀔 수 있음으로 장치 이름을 고정해서 사용할게요.

1)	ttyUSB권한 설정.

USB Port에 대한 권한은 ‘sudo chmod 777 ttyUSB0’ 명령어를 통해서 줄 수 있지만, 매번 설정이 번거로우므로 dialout그룹에 추가하는 방법을 이용.

	sudo usermod -a -G dialout $USER


 
2)	ttyUSB rule의 설정을 위한 심볼릭 링크 ttyUSBx만들기

-	심볼릭 링크를 만드는데 필요한 정보는 아래 3가지.

	Vender ID

	Product ID

	Serial Number

-	위의 3가지 정보는 아래 2개의 명령어로 찾을 수 있어요.

	$ lsusb

	$ udevadm info -a /dev/ttyUSB0 | grep '{serial}'  <-- 저는 처음에 iAHRS가 ttyUSB0으로 잡혀서 이렇게 했어요. 만약 번호가 다르면 그 번호를 입력하세요.
 
![id](https://user-images.githubusercontent.com/58063370/153543765-2284bc56-23ce-4a3d-b261-c855f3ec5089.PNG)
 
위에 출력된 정보가 USB to Serial Device의 Serial Number이며, 해당 번호는 제품별로 다를 수도 있습니다.


-	알아낸 정보를 이용한 .rules file생성.

‘IMU.rules’ 파일을 ‘/etc/udev/rules.d’ 경로에 생성한 후에 해당 파일의 내용을 아래와 같이 작성한 후 저장.

KERNEL=="ttyUSB*", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6015", ATTRS{serial}=="DM03L0C6", MODE:="0666", GROUP:="dialout", SYMLINK+="IMU"

-	udev 재시작 명령어를 호출한 후에 PC의 재 시작을 해주어야 적용 됩니다.

	$ sudo service udev restart


![rules](https://user-images.githubusercontent.com/58063370/153543247-8c446c45-bcab-4ec5-ac96-2550942f5915.PNG)
-	심볼릭 등록의 확인. (아래와 같은 명령어를  호출하면 위의 그림처럼 ttyUSB가 IMU로 고정된 것을 볼 수 있어요 .)

	$ ll /dev/
 

===[사용하기]====================================

1) roslaunch 하기

	$ roslaunch iahrs_driver iahrs_driver.launch 

2) imu topic확인하기

	$ rostopic echo /imu/data 
![222](https://user-images.githubusercontent.com/58063370/153544313-3acc1524-badf-4e7c-af9f-bcf0700ac4e6.PNG)

3) rviz를 사용하여 확인하기

	$ rviz

- rviz화면을 아래와 같이 설정

![rviz_setting](https://user-images.githubusercontent.com/58063370/153546094-76292dd3-dbd8-4e89-9d9c-5242887af163.PNG)

- 동작에 대한 영상은 바로 아래~~~

![ezgif com-crop](https://user-images.githubusercontent.com/58063370/153545614-95801ac6-b86e-4bd2-b46b-3aff10e58a9d.gif)


4) 사용 가능한 서비스 (Input: 없음)
- all_data_reset_cmd : 모든 보정 데이터 초기화(c=9)

- euler_angle_init_cmd: 현재 Euler angle을 0위치로 설정(c=5)

- euler_angle_reset_cmd: Euler angle초기화 (c=7)

- pose_velocity_reset_cmd: 위치와 속도 초기화 (c=8)

- reboot_sensor_cmd: 센서 재시작(c=99)

