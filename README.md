#ITECH ROS IO
> [ICD (institute for computational design)](http://icd.uni-stuttgart.de/?cat=6)<br/>
> [ITKE (institute of building structures and structural design)](http://www.itke.uni-stuttgart.de/index.php?lang=en&id=)<br/>
> [Behrooz Tahanzadeh](http://b-tz.com)<br/>
> nov 2015

Installation
-----
```bash
cd ~/catkin_ws/src/
git clone https://github.com/behrooz-tahanzadeh/itech_ros_io.git
cd ..
catkin_make
```


Usage
-----
```bash
rosrun itech_ros_io topic_io.py <topic name> <ip address> <server port> <optional: minimum interval between messages in seconds>
```
