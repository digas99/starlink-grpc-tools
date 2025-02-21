#!/bin/bash

# check if fleetman/ros:galactic image exists
if [ -z "$(docker images -q fleetman/ros:galactic)" ]; then
	echo "Building fleetman/ros:galactic image"
	sudo docker build -t fleetman/ros:galactic -f deps/ros.dockerfile .
fi


while getopts ":br" arg; do
    case $arg in
    	b)
            BUILD=true
            ;;
		r)
			RUN=true
			;;
    esac
done


# check if starlink/dish_grpc_ros image exists
if [ -z "$(docker images -q starlink/dish_grpc_ros)" ] || [ "$BUILD" = true ]; then
	echo "Building starlink/dish_grpc_ros image"
	sudo docker build -t starlink/dish_grpc_ros -f dish_grpc_ros.dockerfile .
fi

# if not BUILD
if [ -z "$BUILD" ] || [ "$RUN" = true ]; then
	echo "Running starlink/dish_grpc_ros"
	sudo docker run -it --rm --network host --privileged --name dish_grpc_ros starlink/dish_grpc_ros
fi