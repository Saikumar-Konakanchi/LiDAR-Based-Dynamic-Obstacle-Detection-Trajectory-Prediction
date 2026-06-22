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

## Issue log - things that broke and how I found the cause

**No LiDAR collision on the pedestrian**
- Symptom: the LiDAR scan just passed straight through the pedestrian like it wasn't there.
- Root cause: Gazebo Classic's `<actor>` tag has no collision geometry by default.
- Fix: swapped the actor for a regular physics box driven by a `planar_move` plugin, so it actually exists as far as the LiDAR is concerned.

**RViz markers saying "no transform to fixed frame" despite TF looking fine**
- Symptom: markers wouldn't render, but `tf2_echo` and the TF tree showed nothing wrong.
- Root cause: `perception_node` and `tracker_node` were running on wall-clock time while Gazebo was publishing sim time - so the timestamps on outgoing messages didn't line up with what RViz expected.
- Fix: set `use_sim_time` as a default in both nodes so it can't get forgotten again, instead of patching it per-launch.

**Wanted this running across two PCs (sim on one, perception on the other)**
- Symptom: constant ROS2 discovery/DDS issues between WSL2 instances.
- Root cause: WSL2's networking layer doesn't play nicely with ROS2 DDS discovery out of the box, and getting it reliable would've eaten more time than it was worth for this project's scope.
- Decision: kept it on one machine and noted the two-PC setup as a possible follow-up.

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

A low resolution video has been uploaded because of the file size restrictions on GitHub :)
