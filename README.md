Drive hummingbirds  
# 1. Installation:
## 1. Set up ROS, packages for simulation (follow this instruction):    
https://github.com/duynamrcv/hummingbird_simulator  
## 2. Clone this repo 
``` 
cd ~/catkin_ws/src  
git clone https://github.com/trunghieung159/hummingbird_drive 
```   
# 2. Simulation
## 1. Run simulation from launch file   
```
ros launch [package_name] [launch_file] 
```
## 2. Run script
```
ros run [package_name] [script_file_directory] [parameters]  
``` 
## 3. Launch files / Scripts 
- Launch files: 
    - ___One MAV in cave:___ rotors_gazebo/hummingbird_swarm_hovering_example.launch
    - ___Three MAVs in empty space:___ hummingbird_drive/mav_empty.launch
- Script files:
    - ___One MAV point to point:___ hummingbird_drive/point2point.py (PARAMETERS: [x] [y] where (x, y) is goal position)  
    - ___MAVs move to goal:___ hummingbird_drive/formation_control.py (PARAMETERS: [number_of_drones] [x] [y] where (x, y) is leader's goal position)  
