FriendlyARM-Rover
======
FriendlyARM-Rover Unmanned Ground Vehicle (UGV) is an autonomous rover, based in the [FriendlyARM-210 board](http://www.friendlyarm.net/products/smart210).The configuration of the board are as follows, 1GHZ CPU , 512M RAM and 512M Nand Flash.The intelligence of the system comes from interfaces to a GPS sensor, a nertial sensor (mpu9150), a ultrasonic transducer of measuring distance, a USB camera, and a USB WiFi adapter for communication with a base station.The base station now uses [Mission Planner](http://ardupilot.com/downloads/?did=82). Our base station(C#) is under developing. 

We want to make the system have the functions as follows , being controlled by remote server, transfering videos to server, with ability of autonomous navigation  and route planning.The first two part has completed, The other two parts are under developing.

###How to use it?
1. Calibrate nertial sensors 

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
2. Load driver 

```shell
cd ./install
insmod 210pwm.ko

```
3. Compile the whole programe

```shell
cd ./
make

```

4. Run the programe

```shell
#calibrate electronic speed controller
./rover -c 

./rover -p [ip address of server]

```

5. Configuration of base station


6. How to get videos 

