prepare for ros2:
```
distrobox enter ubuntu24

source /opt/ros/jazzy/setup.bash 
colcon build # or colcon build --symlink-install
source install/setup.bash
```


run wrapper node and start services
```
ros2 run zivid_driver zivid_driver_main_test --ros-args -p file_camera_path:=src/zivid-ros/data/FileCameraZivid2PlusLR110.zfc

ros2 run zivid_driver zivid_driver_main_test

```

build service container
new terminal outside distrobox

```
cd ~/flowstate_ws
./src/sdk-ros/scripts/setup_docker.sh

./src/sdk-ros/scripts/build_container.sh \
  --service_package zivid_driver \
  --service_name zivid_driver

./src/sdk-ros/scripts/build_bundle.sh \
  --service_package zivid_driver \
  --service_name zivid_driver
```

deploy in flowstate:
```
export SERVICE_BUNDLE=~/flowstate_ws/images/zivid_driver.bundle.tar 

export INTRINSIC_ORGANIZATION=<org name>

inctl cluster list --org $INTRINSIC_ORGANIZATION

export INTRINSIC_CONTEXT=<cluster id>

inctl service install --org $INTRINSIC_ORGANIZATION --cluster $INTRINSIC_CONTEXT $SERVICE_BUNDLE

```