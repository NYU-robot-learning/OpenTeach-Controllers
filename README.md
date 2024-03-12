# OpenTeach-Controllers

This repository is the collection of symlinks to the controllers/sensor controllers and controller wrappers of robots used for Open-Teach. The robots/sensors include

- [x] [Franka Emika](https://github.com/NYU-robot-learning/OpenTeach-Controllers/tree/main/src/franka-arm-controllers)
- [x] [Kinova Jaco](https://github.com/NYU-robot-learning/kinova-arm-controller-Openteach.git)
- [x] [Xela Tactile sensors](https://github.com/NYU-robot-learning/Xela-Sensor-Controllers-Openteach.git)
- [x] [Allegro Hand Curved](https://github.com/NYU-robot-learning/Allegro-Hand-Curved-Openteach.git)

## Installation

Clone the repository using 

`git clone --recurse-submodules https://github.com/NYU-robot-learning/OpenTeach-Controllers.git`

This will recursively install all the individual controllers 

Follow the instructions in allegro-hand-controllers repository to install CAN drivers. 

For Allegro 

Launch the controller using 

`roslaunch allegro_hand allegro_hand.launch`

For Kinova 

Launch the controller using

`roslaunch kinova_arm kinova_robot.launch`

For Franka

Launch the controller using

`roslaunch franka_arm franka_arm.launch`

For Xela

Launch the controller using

`roslaunch xela_server service.launch` 





