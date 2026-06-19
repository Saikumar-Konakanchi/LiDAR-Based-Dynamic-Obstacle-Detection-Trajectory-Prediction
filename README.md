# LiDAR Rover - Dynamic Obstacle Detection & Tracking

A rover in Gazebo that patrols in a straight line, spots a moving pedestrian using just 2D LiDAR, tracks it with a Kalman filter, predicts where it's going next, and slows/stops if it gets too close.

No SLAM, no Nav2. Just a hand-written perception pipeline.

## Why no SLAM/Nav2

I actually tried SLAM toolbox first. Kept running into TF and odometry issues in WSL2 that weren't going anywhere. Since I didn't actually need mapping or path planning for this (just detecting and reacting to a moving object), I dropped it and built something simpler myself:

- Keep the rover still for a bit, record 100 scans, take the median -> that's the static map.
- Every new scan after that, check each point against the static map. Far from it = dynamic.
- Group nearby dynamic points into clusters.
- Track each cluster with a Kalman filter (constant velocity model).
- Use the velocity estimate to predict where it'll be in 0.5s, 1s, 1.5s, 2s.
- Rover reacts based on distance to the closest tracked object.

## Two nodes

**perception_node.py** - builds the static map, classifies points, clusters them, runs the patrol logic (forward/stop), publishes the HUD and some RViz markers.

**tracker_node.py** - takes the clusters, runs the Kalman filters, does the prediction, publishes the tracked object markers and the "ghost" prediction markers.

Split into two because perception is "what's there" and tracking is "what's it doing." Made debugging way easier.

## Some things that broke along the way

- Gazebo's `<actor>` tag has no collision in Gazebo Classic, so the LiDAR just went straight through the pedestrian. Switched to a regular physics box with a planar_move plugin instead.
- Markers kept showing "no transform to fixed frame" in RViz even though TF looked completely fine. Turned out perception_node and tracker_node weren't using sim time, so their timestamps didn't match Gazebo's clock. Fixed it by setting use_sim_time as a default in both nodes so it can't get forgotten again.
- Originally wanted this running across two PCs (one for sim, one for perception), but WSL2 + Gazebo made that more pain than it was worth for the time I had, so I kept it on one machine.

## Running it

Needs ROS2 Humble + Gazebo Classic, with rover_perception and Lidar_description built in your workspace.

```bash
chmod +x start_project.sh
./start_project.sh
```

It launches Gazebo, waits for the rover to spawn, pauses physics while perception/tracker/RViz come up, then unpauses once everything's ready.

## Files

- `perception_node.py` - static map, classification, clustering, patrol logic
- `tracker_node.py` - Kalman tracking, prediction, markers
- `pedestrian_mover.py` - moves the pedestrian back and forth
- `gazebo.launch.py` - launches gazebo + spawns the rover
- `tracking_world.world` - the world file (walls, boxes, pedestrian)
- `start_project.sh` - runs everything in order

## Video

A low resolution video has been uploaded becasue of the file size restrictions on the GitHub :)
