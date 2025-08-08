# 📏 Benchmarking vS-Graphs

## 📊 Evaluation

To evaluate vS-Graphs against other Visual SLAM systems (e.g., ORB-SLAM3), follow the steps below:

### 🔧 1. Run Pose Storage Script

Run [`generate_kf_pose_txt_files.py`](../evaluation/generate_kf_pose_txt_files.py) to store estimated robot poses (based on the KeyFrames). Make sure that you set proper configuration parameters beforehand in the [`config.yaml`](../evaluation/config.yaml) file. The script will create a `.txt` file and populate it with pose data:

```python
python [workspace]/src/visual_sgraphs/evaluation/generate_kf_pose_txt_files.py
```

### ▶️ 2. Run vS-Graphs

Launch vS-Graphs and play the `ros2 bag` file. Pose data will be recorded automatically into the generated `.txt` file during execution.

### 📈 3. Run Evaluation

Once both ground-truth (such as LiDAR S-Graphs) and estimated poses (the generated `.txt` file) are ready, use [`evo`](https://github.com/MichaelGrupp/evo) to compute Absolute Pose Error (APE):

```bash
evo_ape tum [gt_pose].txt [vs_graphs_pose].txt -va > results.txt --plot --plot_mode xy
```

## 🗺️ Working with Maps

vS-Graphs uses `.osa` files to store maps. These files are saved by default in your `ROS_HOME` directory (`~/.ros/`).

---

### 🔄 Loading a Map

- To load a saved map, set the `System.LoadAtlasFromFile` parameter in your SLAM settings YAML file.
- ⚠️ If no `.osa` file is available, **comment out** the `System.LoadAtlasFromFile` parameter to avoid runtime errors.

---

### 💾 Saving a Map

- **Option 1**: Enable `System.SaveAtlasToFile` in the SLAM settings file. The map will be automatically saved when you shut down the ROS node.
- **Option 2**: Manually trigger a save using the following ROS service:

```bash
rosservice call /vs_graphs/save_map [file_name]
```

### 🛠️ Map and Trajectory ROS Services

Use the following ROS services to save maps and trajectories during or after running vS-Graphs:

- To save the current SLAM map to `~/.ros/[file_name].osa` use `rosservice call /vs_graphs/save_map [file_name]`.
- To exports the estimated trajectories use `rosservice call /vs_graphs/save_traj [file_name]`
