FriendlyARM-ROV
======
FriendlyARM-ROV is a Remotely-Operated Vehicle (ROV), based in the [FriendlyARM-210 board](http://www.friendlyarm.net/products/smart210).The configuration of the board are as follows, 1GHZ CPU , 512M RAM and 512M Nand Flash.The intelligence of the system comes from interfaces to a GPS sensor, a nertial sensor (mpu9150), a ultrasonic transducer of measuring distance, a USB camera, and a USB WiFi adapter for communication with a base station.The base station now uses [Mission Planner](http://ardupilot.com/downloads/?did=82). Our base station(C#) is under developing. 

We want to make the system have the functions as follows: 
- Being controlled by remote server
- Transfering videos to server
- With ability of autonomous navigation
- Route planning

###Getting the source
You can either download the source using the “ZIP” button at the top of the github page, or you can make a clone using git:

```shell
git@github.com:FongLee/rover.git
```

###Prerequisites
####Ubuntu Linux
To build FriendlyARM-ROV,you'll first need to install cross compilation tool chain of arm, you can download it from [Friendlyarm Official Website](http://www.friendlyarm.net/downloads).
####Building using make
#####Calibrate nertial sensors 
At first, nertial sensors (acceleration and magnetic sensor)should be calibrated, beacause there are different errors in different places.Especially, magnetic sensor has a large error, which  is easily influenced by metal.

```shell
#./ means the current directory of rover
cd ./lib/ap_imu_sensor/examples
make
#This means calibrating Acceleration Sensor. Turn the rover to six 
#different  directions.In every directions, you should input 'Enter'
# key, which makes the sensor collects data.After collecting the data
# of six defferent directions, it will create a file named accelcal.txt.
./imucal -a
#This means calibrating Magnetic Sensor . Turn the rover to six 
#different  directions.In every directions, you should input 'Enter'
# key, which makes the sensor collects data.After collecting the data 
# of six defferent directions, it will create a file named magcal.txt.
./imcal -m

```
#####Load driver 
It is a driver of PWM in board to driver electronic speed controller.

```shell
cd ./install
insmod 210pwm.ko

```
#####Compile the whole programe

```shell
cd ./
make

```

###Usage of programe
Before you run the whole system, calibration of electronic speed controller is necessary.Otherwise, The electronic speed controller don't know which pwm is high and low.

```shell
#calibrate electronic speed controller first
./rover -c 

./rover -p [ip address of server]

```


