# iAHRS ROS Package

===[Device Setting]====================================

-	HerkuleX와 PC간의 통신 연결을 위한 시리얼 인터페이스 장치와 전원공급 장치를 준비하여 아래 그림과 같이 연결합니다.

 ![0](https://user-images.githubusercontent.com/58063370/73432722-addf6980-4386-11ea-8a9b-e4104ebc70d8.png)
 
그림 1. HerkuleX와 PC간의 연결 설명서

	요즘 PC에는 대부분 Serial Port가 없기 때문에 위의 그림과 같이 USB to Serial Device를 추가로 사용해야 합니다.

	이 USB to Serial Device를 이용하여, PC와 HerkuleX간의 RS232통신을 하기 때문에 Ubuntu환경에서는 ttyUSB권한 설정 및 USB rule설정을 초기에 1회 해주어야 하며, 그 방법은 다음과 같습니다.

1)	ttyUSB권한 설정.

HerkuleX의 USB Port에 대한 권한은 ‘sudo chmod 777 ttyUSB0’ 명령어를 통해서 줄 수 있지만, 매번 설정이 번거로우므로 dialout그룹에 추가하는 방법을 이용한다.

	sudo usermod -a -G dialout $USER

 * 위의 명령어를 실행 후 PC의 재 시작을 해야 적용된다.
 
2)	ttyUSB rule의 설정을 위한 심볼릭 링크 ttyUSBx만들기

-	심볼릭 링크를 만드는데 필요한 정보는 아래 3가지 이다.

	Vender ID

	Product ID

	Serial Number

-	위의 3가지 정보는 아래 2개의 명령어로 알아낼 수가 있다.

	$ lsusb

![1](https://user-images.githubusercontent.com/58063370/73432967-2b0ade80-4387-11ea-9acc-6bad239121a9.png)

그림 2. USB list

	$ udevadm info -a /dev/ttyUSB0 | grep '{serial}'
 
![001](https://user-images.githubusercontent.com/58063370/73433144-85a43a80-4387-11ea-80a1-65c0a1facccb.png)
 
그림 3. USB to Serial Device Serial Number

위에 출력된 정보가 USB to Serial Device의 Serial Number이며, 해당 번호는 제품별로 상이하다

-	알아낸 정보를 이용한 .rules file생성.

![3](https://user-images.githubusercontent.com/58063370/73433039-4ece2480-4387-11ea-8d70-babf577869ce.png)

그림 4. USB rules file

‘HerkuleX.rules’ 파일을 ‘/etc/udev/rules.d’ 경로에 생성한 후에 해당 파일의 내용을 아래와 같이 작성한 후 저장한다.

KERNEL=="ttyUSB*", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6001", ATTRS{serial}=="AK05VTK7", MODE:="0666", GROUP:="dialout", SYMLINK+="HerkuleX"

저장완료 후에는 아래와 같은 명령어를 이용하여 작성 내용의 확인이 가능하다.

![2](https://user-images.githubusercontent.com/58063370/73433002-36f6a080-4387-11ea-9cd2-41d500dbb968.png)
 
그림 5. HerkuleX.rules file확인

-	udev 재시작 명령어를 호출한 후에 PC의 재 시작을 해주어야 적용이 된다.

	$ sudo service udev restart

-	심볼릭 등록의 확인. (아래와 같은 명령어를 이용하여, 심볼릭 링크로 설정한 ttyUSB장치가 tetra로 적용되었는지 여부를 확인한다.)

	$ ll /dev/
 
![4](https://user-images.githubusercontent.com/58063370/73433186-9b196480-4387-11ea-9929-d0e0f0a623e8.png)
 
그림 6. Device List확인


