# OpenTeach-Controllers

This repository is the collection of symlinks to the controllers/sensor controllers and controller wrappers of robots used for [Open-Teach](https://arxiv.org/abs/2403.07870). The robots/sensors include

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


### Citation
If you use this repo in your research, please consider citing the paper as follows:
```@misc{iyer2024open,
      title={OPEN TEACH: A Versatile Teleoperation System for Robotic Manipulation}, 
      author={Aadhithya Iyer and Zhuoran Peng and Yinlong Dai and Irmak Guzey and Siddhant Haldar and Soumith Chintala and Lerrel Pinto},
      year={2024},
      eprint={2403.07870},
      archivePrefix={arXiv},
      primaryClass={cs.RO}
}


