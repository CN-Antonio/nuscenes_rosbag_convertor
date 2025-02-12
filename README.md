reference

- [nuscenes_visualize](https://github.com/Owen-Liuyuxuan/nuscenes_visualize)  
- [rclpy Params Tutorial – Get and Set ROS2 Params with Python](https://roboticsbackend.com/rclpy-params-tutorial-get-set-ros2-params-with-python/)  
- [nuscenes2bag](https://github.com/foxglove/nuscenes2bag)

dependence
https://github.com/nutonomy/nuscenes-devkit/blob/master/docs/installation.md
- python==3.7
``` bash
# (nuscenes)
conda create -n nuscenes python=3.7
conda activate nuscenes
pip install nuscenes-devkit
export PYTHONPATH=/opt/ros/humble/lib/python3.10/site-packages:$HOME/miniconda3/envs/nuscenes/lib/python3.7/site-packages

```
- v1.0-mini.tgz
- Map expansion pack
```
nuscenes
├── Full_dataset_v1.0
    └── v1.0-mini
        ├── can_bus -> ../can_bus/
        ├── maps
        │   ├── 36092f0b03a857c6a3403e25b4b7aab3.png
        │   ├── 37819e65e09e5547b8a3ceaefba56bb2.png
        │   ├── 53992ee3023e5494b90c316c183be829.png
        │   ├── 93406b464a165eaba6d9de76ca09f5da.png
        │   ├── basemap -> ../../../nuScenes-map-expansion-v1.3/basemap/
        │   ├── expansion -> ../../../nuScenes-map-expansion-v1.3/expansion/
        │   └── prediction -> ../../../nuScenes-map-expansion-v1.3/prediction/
        ├── samples
        ├── sweeps
        └── v1.0-mini
├── can_bus
└── nuScenes-map-expansion-v1.3
```

run
```bash

```