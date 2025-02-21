FROM fleetman/ros:galactic

# Install Python3 and pip
RUN apt-get update && \
    apt-get install -y python3 python3-pip

RUN python3 -m pip install --upgrade pip

# Install ROS 2 dependencies
RUN apt-get update && \
    apt-get install -y \
    python3-rosdep \
    python3-rosinstall-generator \
    python3-vcstool \
    build-essential

# Initialize rosdep
RUN rosdep init && \
    rosdep update

# Install rclpy
RUN apt-get update && \
apt-get install -y ros-galactic-rclpy

# Install required Python packages
COPY requirements.txt /app/
RUN pip install --upgrade -r /app/requirements.txt

COPY dish_grpc_ros.py \
	dish_common.py \
	starlink_grpc.py \
	ros/entrypoint.sh \
	/app/
WORKDIR /app


ENTRYPOINT ["/app/entrypoint.sh"]
CMD ["dish_grpc_ros.py"]