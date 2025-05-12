# ROS Path Planner

This project implements a basic but robust **A\* path planner** using ROS and C++. It computes the shortest path from a given start position to a goal in a 2D occupancy grid map while avoiding obstacles.

The path and important positions (start, goal) are visualized in RViz using colored markers.



---

## 🧰 Dependencies

- ROS Noetic or Melodic
- `map_server`
- RViz
- C++14 or later

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

## 🛠️ Building the Package

Open a terminal and run:

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

---

## 🚀 Launching the Planner

Run the complete system (map + planner + RViz):

```bash
roslaunch simple_path_planner planner.launch
```

You can optionally change the obstacle penalty like this:

```bash
roslaunch simple_path_planner planner.launch p:=1500
```

---

## 🧪 Using the Planner in RViz

1. Launch the system using the command above.
2. In RViz:
   - Set **Fixed Frame** to `map`
   - Add:
     - `Map` (topic: `/map`)
     - `Marker` (topic: `/visualization_marker`)
   - Use the **2D Pose Estimate** tool to set the start pose
   - Use the **2D Nav Goal** tool to set the goal pose
3. The planned path will appear as a colored line, and spheres will mark the start and goal positions.

---

## 🎨 Marker Colors

| Element       | Color    |
|---------------|----------|
| Start Point   | 🟡 Yellow |
| Goal Point    | 🔴 Red    |
| Path Line     | 🔷 Cyan   |

---

## 🔍 Heuristic

This planner uses **Euclidean distance** as the heuristic metric for A*.

---

## 🖼️ Simulation Screenshot

> ![Simulation](docs/simulation_example.png)

Place your screenshot in the `docs/` folder with this exact filename: `simulation_example.png`.

---

## 📦 Commands Summary (Copy & Paste Friendly)

### 🧱 Build and Source

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

### 🚀 Launch Planner

```bash
roslaunch simple_path_planner planner.launch
```

### 🧠 Optional: Launch with Custom Obstacle Penalty

```bash
roslaunch simple_path_planner planner.launch p:=2000
```

---

## 📬 Contact

Feel free to fork the repository or open issues for bugs and improvements.

---

## 📄 License

This project is open source and released under the **MIT License**.




