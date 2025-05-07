# ORB_SLAM3 docker

This docker is based on <b>Ros Noetic Ubuntu 20</b>. If you need melodic with ubuntu 18 checkout #8fde91d

There are two versions available:
- CPU based (Xorg Nouveau display)
- Nvidia Cuda based. 

To check if you are running the nvidia driver, simply run `nvidia-smi` and see if get anything.

Based on which graphic driver you are running, you should choose the proper docker. For cuda version, you need to have [nvidia-docker setup](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html) on your machine.

---

## Compilation and Running

Steps to compile the Orbslam3 on the sample dataset:

- `./download_dataset_sample.sh`
- `build_container_cpu.sh` or `build_container_cuda.sh` depending on your machine.

Now you should see ORB_SLAM3 is compiling. 
- Download A sample MH02 EuRoC example and put it in the `Datasets/EuRoC/MH02` folder
```
mkdir -p Datasets/EuRoC 
wget -O Datasets/EuRoC/MH_02_easy.zip http://robotics.ethz.ch/~asl-datasets/ijrr_euroc_mav_dataset/machine_hall/MH_02_easy/MH_02_easy.zip
unzip Datasets/EuRoC/MH_02_easy.zip -d Datasets/EuRoC/MH02
```
To run a test example:
- `docker exec -it orbslam3 bash`
- `cd /ORB_SLAM3/Examples && bash ./euroc_examples.sh`
It will take few minutes to initialize. Pleasde Be patient.
---

You can use vscode remote development (recommended) or sublime to change codes.
- `docker exec -it orbslam3 bash`
- `subl /ORB_SLAM3`

git clone -b add_euroc_example.sh https://github.com/jahaniam/ORB_SLAM3.git /ORB_SLAM3 
cd /ORB_SLAM3
chmod +x build.sh
./build.sh


# Add ROS package path to bashrc
echo 'export ROS_PACKAGE_PATH=/opt/ros/noetic/share:/ORB_SLAM3/Examples/ROS' >> ~/.bashrc

# Source the updated bashrc
source ~/.bashrc


# Make the build script executable
chmod +x build_ros.sh

catkin build
source ~/catkin_ws/devel/setup.bash

File.version: "1.0"
Camera.type: "PinHole"
Camera calibration and distortion parameters (OpenCV)
Camera1.fx: 644.6349487304688
Camera1.fy: 643.6536254882812
Camera1.cx: 637.49169921875
Camera1.cy: 361.69696044921875
Camera1.k1: -0.05639747902750969
Camera1.k2: 0.06732191890478134
Camera1.p1: -0.0003759726241696626
Camera1.p2: 0.0009479765431024134
Camera.width: 1280
Camera.height: 720
Camera frames per second
Camera.fps: 20
Color order of the images (0: BGR, 1: RGB. It is ignored if images are grayscale)
Camera.RGB: 1
Transform from camera to IMU
Camera1.T_cam_imu: !!opencv-matrix
rows: 4
cols: 4
dt: f
data: [1.0, 0.0, 0.0, -0.005,
0.0, 1.0, 0.0, 0.017,
0.0, 0.0, 1.0, 0.022,
0.0, 0.0, 0.0, 1.0]


