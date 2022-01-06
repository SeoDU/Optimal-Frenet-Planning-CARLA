# Carla_optimal_frenet_planning
1. Optimal frenet path planning
2. PID controller considering curvature and expected path
3. Additional Obstacle detection using 2 algorithms Multi-Object-Tracking, Patchwork for lidar.

![overall_architecture](https://user-images.githubusercontent.com/47074271/148345732-9d2d8321-f7ca-4c07-bdd1-bffa66f4345e.png)

Final implementation : https://www.youtube.com/watch?v=zuEOWpu27rI    

## How to run
```
bash ~/CARLA_0.9.10.1/CarlaUE4.sh
rosrun carla_ad_agent spawn_npc.py
roslaunch carla_ad_demo carla_ad_demo_with_rviz.launch
```

## Additional obstacle Detection
- Patchwork : https://github.com/LimHyungTae/patchwork
- Multiple-object-tracking-lidar Package : https://github.com/praveen-palanisamy/multiple-object-tracking-lidar
```
roslaunch patchwork rosbag_kitti.launch
rosrun multi_object_tracking_lidar kf_tracker filtered_cloud:=/benchmark/N
```

## Reference
- https://github.com/carla-simulator/carla
- https://github.com/carla-simulator/ros-bridge
- https://tonylim.tistory.com/63
- https://www.researchgate.net/publication/224156269_Optimal_Trajectory_Generation_for_Dynamic_Street_Scenarios_in_a_Frenet_Frame
