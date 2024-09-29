FROM docker.1panel.live/osrf/ros:noetic-desktop-full
# Set the environment variable to non-interactive, this is to avoid the tzdata configuration during the installation of the packages
ENV DEBIAN_FRONTEND=noninteractive
RUN sed -i 's#http://archive.ubuntu.com/#http://mirrors.tuna.tsinghua.edu.cn/#' /etc/apt/sources.list

# System layer installation
RUN apt-get update && \
    apt-get upgrade -y && \
    apt-get install -y wget && \
    apt-get install -y iputils-ping && \
    apt-get -y install build-essential && \
    apt install -y cmake && \
    apt install -y tree && \
    apt install -y gdb && \
    apt-get install -y git && \
    apt-get install -y python3-pip && \
    apt-get install -y xauth x11-apps # This is for GUI testing

# Dependencies layer # Ur_driver
RUN mkdir -p /catkin_ws/src && \
    cd /catkin_ws/src && \
    git clone -b noetic-devel https://github.com/ros-industrial/universal_robot.git && \
    # git clone https://github.com/UniversalRobots/Universal_Robots_ROS_Driver.git && \
    apt-get update && \
    apt-get upgrade -y && \
    apt install -y ros-noetic-ur-robot-driver && \
    apt-get install -y ros-${ROS_DISTRO}-moveit-planners-chomp && \
    apt-get install -y ros-${ROS_DISTRO}-pilz-industrial-motion-planner && \
    apt-get install -y ros-${ROS_DISTRO}-moveit-ros-control-interface && \
    apt-get install -y ros-${ROS_DISTRO}-moveit-commander

# Robotiq gripper (gripper model, gazebo ros control plugin and usb communication interfaces)
RUN mkdir -p /catkin_ws/src && \
    cd /catkin_ws/src && \
    git clone https://github.com/filesmuggler/robotiq.git && \
    cp -r robotiq/robotiq_description/  .&& \
    rm -rf robotiq && \
    git clone https://github.com/roboticsgroup/roboticsgroup_gazebo_plugins && \
    git clone https://github.com/TAMS-Group/robotiq.git && \
    apt-get install -y ros-noetic-soem && \
    apt-get install ros-noetic-socketcan-interface && \
    apt install ros-noetic-rqt-controller-manager && \
    apt-get install -y python3-tk && \
    apt-get install -y ros-noetic-ros-control ros-noetic-ros-controllers

# Dependencies layer # PCL and VTK and Pyvista and open3d
RUN apt-get update -y && \
    apt-get install -y libvtk7-dev python3-vtk7 && \
    apt install -y libpcl-dev &&\
    apt install -y python3-pcl &&\
    pip3 config set global.index-url https://pypi.tuna.tsinghua.edu.cn/simple && \
    pip3 install numpy vtk &&\
    pip3 install pyvista && \
    pip3 install pyvistaqt meshio &&\
    pip3 install transforms3d && \
    pip3 install scipy

# Dependencies layer # Azure Kinect SDK
RUN wget https://packages.microsoft.com/ubuntu/18.04/prod/pool/main/libk/libk4a1.3/libk4a1.3_1.3.0_amd64.deb && \
    wget https://packages.microsoft.com/ubuntu/18.04/prod/pool/main/libk/libk4a1.3-dev/libk4a1.3-dev_1.3.0_amd64.deb && \ 
    wget https://packages.microsoft.com/ubuntu/18.04/prod/pool/main/k/k4a-tools/k4a-tools_1.3.0_amd64.deb && \
    echo 'libk4a1.3 libk4a1.3/accepted-eula-hash string 0f5d5c5de396e4fee4c0753a21fee0c1ed726cf0316204edda484f08cb266d76' | debconf-set-selections && \
    dpkg -i libk4a1.3_1.3.0_amd64.deb && \
    dpkg -i libk4a1.3-dev_1.3.0_amd64.deb && \
    apt-get install -y libsoundio1 && \
    dpkg -i k4a-tools_1.3.0_amd64.deb
    
# Dependencies layer # OpenCV and find_object_2d
RUN apt-get update -y && \
    apt-get install -y ros-noetic-find-object-2d &&\
    pip3 install opencv-contrib-python==4.8.0.76 && \
    pip3 install pyk4a && \
    pip3 install open3d && \
    pip3 install cv_bridge

# Clone my custimized repositories
RUN cd /catkin_ws/src && \
    git clone https://github.com/Jaron-G/robotiq_gripper.git && \
    git clone https://github.com/Jaron-G/ur10e_gripper.git && \
    git clone https://github.com/Jaron-G/ur10e_gripper_moveit.git

# Use rosdep to install all dependencies according to the package.xml files in all src folders
# And compile the workspace
RUN cd /catkin_ws && \
    apt-get update && \
    rosdep install --from-paths src --ignore-src -r -y && \
    /bin/bash -c '. /opt/ros/noetic/setup.bash; catkin_make'

# Automatically source
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc && \
    echo "source /catkin_ws/devel/setup.bash" >> /root/.bashrc

RUN echo "ALL Done"
