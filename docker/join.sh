#!/bin/bash

CONTAINER_NAME="ros2_humble_andino_fleet_container"
echo "Using Container Name: $CONTAINER_NAME"
docker exec -it "$CONTAINER_NAME" bash
