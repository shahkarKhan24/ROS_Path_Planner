# ROS Path Planner

This project implements a basic but robust **A\* path planner** using ROS and C++. It computes the shortest path from a given start position to a goal in a 2D occupancy grid map while avoiding obstacles.

The path and important positions (start, goal) are visualized in RViz using colored markers.
---

## Dependencies

- ROS Noetic or Melodic
- `map_server`
- RViz
- C++

---

## 📁 Project Structure

```
simple_path_planner/
├── launch/
│   └── planner.launch
├── maps/
│   └── map.yaml + map.pgm
├── src/
│   └── refactored_astar_planner.cpp
├── docs/
│   └── simulation_example.png
├── CMakeLists.txt
└── package.xml
```

---

## 🛠Building the Package

Open a terminal and run:

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
cd src/
cd simple_path_planner/
```

---

##  Launching the Planner

Run the complete system, all necessary files that are needed to launch ar being run by single lanunch file (map + planner + RViz), :

```bash
roslaunch simple_path_planner planner.launch
```



---

## Using the Planner in RViz

1. Launch the system using the command above.
2. In RViz:
   - Set **Fixed Frame** to `map`
   - Add:
     - `Map` (topic: `/map`)
     - `Marker` (topic: `/visualization_marker`)
   - Use the **2D Pose Estimate** tool to set the start pose
   - Use the **2D Nav Goal** tool to set the goal pose
---

## 🖼️ Simulation Screenshot

> ![Simulation](docs/simulation_example.png)

Place your screenshot in the `docs/` folder with this exact filename: `simulation_example.png`.

---



## 🔍 Heuristic

This planner uses **Euclidean distance** as the heuristic metric for A*. But as an extension of this project, the code can be edited to use other heuristic metrics







