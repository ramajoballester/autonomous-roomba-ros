# Robotics
Autonomous Navigation framework for mobile robots in ROS and Gazebo

## Installation

Clone this repository in your workspace

```
git clone https://github.com/ramajoballester/robotics.git
```

Install the python packages required

```
cd robotics/
sudo -H pip install -r requirements.txt
```


## Examples

### SLAM mapping launch file

``` bash
roslaunch robotics mapping.launch
```

### Autonomous navigation

```
roslaunch robotics autonomous.launch
```


### Autonomous multi-robot navigation and task allocation.

```
roslaunch robotics autonomous_multi_house.launch
```


