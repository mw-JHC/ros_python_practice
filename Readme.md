# ros_practice with Python3!
Keyword:
python, node, topic, publisher, subscriber, loop
***
## ROS and Python cheat sheet for my self
This material is intended as a review of the ROS practice for my self.  
Most contents are originally from Udemy lecture "ROS For Beginners (ROS Noetic, Melodic, Kinetic)" by Edouard Renard.
***
### Create Catkin workspace, ROS Package and Nodes with Python3 
Enviroment: WSL, Unbuntu 20.04, ROS-Noetic, Python3
***
### 1. Catkin Workspace
* What is a catkin workspace?  
Catkin is the official bulit system for ROS  
* How to a create catkin workspace?
  ```console  
  $ mkdir catkin_ws
  $ cd catkin_ws
  $ mkdir src
  $ catkin build # or catkin_make but catkin_make is old one
  $ source devel/setup.bash
  ```  
  Catkin_make is the command to build the ROS Workspace.
***
### 2. ROS Package
* What is a ROS Package?  
ROS Package will allow us to separate our code into resusable block so the development and maintenance of the application will be easier
* How to create a ROS Package?
  ```console
  $ cd catkin_ws/src
  $ catkin_create_pkg {name of a package} {dependencies of the package}
  ```
  for example
  ```console
  $ catkin_create_pkg robot_package roscpp rospy std_msgs
  ```
  it mean we create a ROS Package with the dependencies roscppp(for c++), rospy(for Python) and std_msgs(the basic ROS package Standard Messages
***
### 3. ROS Node
* What is a ROS Node?(according to ROS Wiki)
  A ROS node is a process that performs computation. Nodes are combined together into a graph and communicate with one another using streaming topic, RPC services, and the Parameter Server. 

to use python3 in ROS noetic, we must write this comment in the first line for python interpreter.
```python
#!/usr/bin/env python3
```

* how to create ros node?
```python
import rospy

rospy.init_node("node name")
```
* anonymous node
```python
import rospy

rospy.init_node("node name", anonymous = True)
```
(see __src/practice_1__)
***
### 4. ROS Topic Publisher and Subscriber
ROS node can communicate each other through topic.
A node publish a Topic, we call the node as a publisher
A node subscribe a topic, we call the node as a subscriber
A Topic consist of message!

* how to define a publisher?
```python
import rospy
import message.msg

# init a Node
rospy.init_node("node_name")

# declare a publisher
pub = rospy.Publisher("topic_name", message.msg.Datatype, queue_size = int_number)

# declare a message
msg = message.msg.Datatype()
msg.data = contents_of_message 
pub.publish(msg)

# or we can do like
from mssage.msg import Datatype
pub = rospy.Publisher("topic_name", Datatype, queue_size = int_number)
```

* how to define a subscriber
for a subscriber we need to define a callback function
```python
import rospy
import message.msg

def callback_sub(msg):
  rospy.loginfo(msg)

rospy.init_node('node_name')
sub = rospy.Subscriber("topic_name", message.msg.String, callback_sub)
```
(see __src/practice_topic__)
***
### 5. ROS Service
* how to use ros Service
a service file look like ...
```
# request
int64 a
int64 b
---
# result
int64 sum
```
```python
import rospy_tutorial.srv

def handle_function(req):
  result = req.a + req.b 
  return result

service = rospy.Service("/add_two_ints", rospy_tutorials.srv.example, handle_function)
```
(see __src/practice_service/add_two_ints_server.py__)
***
### 6. ROS custom msg and srv
we can create a customized ROS-message or ROS-service.

* how to create a ROS custom message?
you must create a ROS package with the catkin_create_pkg command.  
After that you must add some code in package.xml and CMakeLists.txt!

#### 6.1 create a customized message

in package.xml

```xml
<build_depend>message_generation</build_depend>
<exec_depend>message_runtime</exec_depend>
```

in CMakeList.txt
message_generation is added in find_package and generate_messages must be uncommented. Additionally you must add message_runtime under caktin_package.

__For example__:
```
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES practice_custom_msg_srv
 CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
#  DEPENDS system_lib
)
```

and create a new directory as name msg for custom message.  
reference: http://wiki.ros.org/msg

you can see the a example customized message in the folder __msg__ under practice_custom_msg_srv. To create a customized message the folder name __msg__ is very important.

__For example__
```
int64 temperature
bool are_motors_up
string debug_message
```

After creating a custom message we must change some codes in the CMakeLists.txt once again.

__For example__
```txt
add_message_files(
  FILES
  YourCustomizedMessage.msg
)
```
now you can build the package with catkin build. After the Building you can find your customized message in the folder devel/include/your_package_name
(see src/practice_custom_msg_srv)

* how to use a customized ROS message?

for practice we create a new ros package.

```console
catkin_create_pkg your_customized_package roscpp rospy std_msgs std_srvs rospy_tutorials
```

and to use our custom message we must add this command in package.xml
```xml
<depend>your_customized_package</depend>
```

and add your package in CMakeList.txt
```txt
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  rospy_tutorials
  std_msgs
  std_srvs
  your_customized_package
)
```

after that... using method is same like how to use ROS message

(see __src/practice_using_custom_msg_srv__)

#### 6.2 create a customized service
you must create fist a folder named __srv__ under a ROS packages like __msg__ to use your customized message.

A ROS service consists of request and response.(see chapter 5). now we create with this format a new customized service.

__For example__
```txt
#request
float64 radius
---
#response
float64 area
```

After creating a new customized service. we must change some codes in the CMakeLists.txt once again.

__For example__
```txt
add_service_files(
  FILES
  ComputeDiskArea.srv
)
```

Actually we need to change some codes too in package.xml but if you follow this practice you did it in section 6.1.
but i wrote once aging for you, who will see this Readme in the future.
The package.xml must include this codes.
```xml
<build_depend>message_generation</build_depend>
<exec_depend>message_runtime</exec_depend>
```
```xml
<depend>your_customized_package</depend>
```

Now you can see the that the customized service is created under devel/include/your_package_name after catkin build command.  
You can see that only one header file is create for a customized message for example HardwareStatus.h but three header files are create for a customized service for example ComputeDiskArea.h, ComputerDiskAreaRequest.h and ComputeDiskAreaResponse.h

after that... using method is same like how to use ROS service
(see src/practice_using_custom_msg_srv and src/practice_service)

__more practice example__  
you can see more examples to use a customized service in __practice_custom_msg_srv/srv/SetLed.srv__ and __practice_service/scripts/led_panel.py__ and __practice_service/scripts/battery.py__

and you can use your ros service with the command
```console
rosservice call /your_service <tab tab>
```

### 7. ROS Parameter
ROS parameter server will be started automatically, when roscore is started.  

#### 7.1 ROS parameter command line tools
```console
rosparam -h
rosparam list
rosparam set </your_parameter_name> <value>
rosparam get </your_parameter_name>
```

#### 7.2 ROS parameter handle in python
```python
rospy.set_param("/number_publish_frequency", 1)
value_name = rospy.get_param("/your_parameter_name")
```
(see __src/practice_ros_param/scripts/number_publisher_with_ros_param.py__ to understand rosparma compare the __src/pratice_topic/scripts/number_publisher.py__)
***
### 8. ROS Launch
ROS launch allow you start every nodes and with parameters. 

we create fist a new package for a launch file. For ROS launch we don't need any ros dependencies cause we can create with this command a ros packages easily. But if you create some scripts or src in the package with c++ or python you must create a package with ros dependencies, which you need.
```console
catkin_create_pkg <your_package_name>
```

and you should a launch directory under the package, which you created for launch. And under the launch folder you should create a file for example your_launchfile.launch

#### 8.1 ROS launch file
ROS launch file is created with xml format. 

__For example__
```xml
<launch>
    <param name = "/your_ros_parameter" type = "type_of_parameter" value = "value_of_parameter"/>
    <!-- Example -->
    <param name = "/number_to_bulish" type = "int" value = "2"/>

    <node name = "Your_node_name" pkg = "Where_your_script_located" type = "your_script_name"/>
    <!-- example -->
    <node name = "number_counter" pkg = "practice_topic" type = "number_counter.py"/>
</launch>
```

#### 8.2 ROS launch command line
```console
roslaunch <your_package_name> <your_launch_file>
```

(see __src/practice_ros_launch/launch__)
***
### 9. ROS Bag
you can record your topics with ROS bag functionality.

```console
rosbag record <your_topic_name>
rosbag info <your_ros_bag_file >
rosbag play <your_ros_bag_file> 
# you can check with rostopic echo <your_ros_topic_name_what_you_recorded>
```
***

### 10. OOP
you know what is oop and it is really important to create a big python project. 

you can create a ros package as oop too.

To understand how to create ros python package as oop plz compare this files

__/src/practice_topic/scripts/number_counter_as_oop.py__
__/src/practice_topic/scripts/number_counter_not_as_oop.py__