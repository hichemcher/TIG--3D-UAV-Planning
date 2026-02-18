
# TIG-Star-3D-UAV-Planning

Official MATLAB implementation of:

**TIG\***: *Enhanced Tangent Intersection Guidance for Efficient 3D UAV Path Planning in Complex Environments*  
H. Cheriet, B. Khellat Kihel, S. Chouraqui  
IEEE Open Journal of Vehicular Technology, 2026  
DOI: https://doi.org/10.1109/OJVT.2026.3659786  

---

## 🚀 Overview

TIG-Star-3D-UAV-Planning is a geometric, graph-based 3D path planning framework for Unmanned Aerial Vehicles (UAVs) operating in complex environments.

This repository contains the complete implementation of:

- **S-TIG\*** — Static Tangent Intersection Guidance (known environments)  
- **O-TIG\*** — Online Tangent Intersection Guidance (unknown environments with local sensing)  

Both planners are presented in the IEEE OJVT 2026 paper.

The framework is designed for:

- Research reproducibility  
- Performance benchmarking  
- Extension toward intelligent and AI-enhanced planners  
- Academic and PhD-level research experimentation  

---

## 🚀 Algorithmic Concept

TIG\* is based on structured geometric reasoning rather than dense random sampling.

Core principles include:

- Tangent-based waypoint generation  
- A*-like guided expansion  
- First-intersection obstacle detection  
- UAV smoothness constraints (turn-angle limitation)  
- Obstacle-top traversal strategy  

### 🔹 S-TIG\* (Static)

Designed for fully known 3D environments.  
The planner computes an efficient collision-free path using tangent intersection logic.

### 🔹 O-TIG\* (Online / Unknown)

Designed for partially known or unknown environments.  
The planner operates with:

- Limited sensing radius  
- Iterative local replanning  
- Adaptive sensing expansion  
- Escape behavior when local planning fails  

O-TIG\* extends TIG\* toward realistic UAV navigation scenarios.

---

## ⚙ Requirements

- MATLAB R2023a  
- No additional toolboxes required  
- Tested on Windows and macOS  

---

## ▶ How to Run

### Run Static TIG\*

Open:

```
main/run_static.m
```

Then execute:

```matlab
run_static
```

---

### Run Online TIG\*

Open:

```
main/run_unknown.m
```

Then execute:

```matlab
run_unknown
```

---

## 📊 Performance Metrics

The framework evaluates:

- Path Length  
- Turning Radius  
- Smoothness (angle-based constraint)  
- Computation Time  

Metrics are computed using:

```matlab
compute_path_metrics(path, ds)
```

---

## 🚀 Environment Model

The planner operates in bounded 3D environments containing:

- Polygonal prism obstacles (extruded 2D polygons)  
- Arbitrary map dimensions  
- UAV modeled as a point with configurable safety margin  

Collision checking includes:

- Line–polygon intersection tests  
- Height constraint validation  
- Obstacle-top traversal logic  

---

## 🎥 Visualization

3D visualization tools are included:

```matlab
display_environment_3d(env, results, options);
```

Optional UAV animation:

```matlab
animate_path(path, anim_options);
```

---

## 📚 Citation

If you use this work in your research, please cite:

```bibtex
@ARTICLE{Cheriet2026TIG,
  author={Cheriet, Hichem and Khellat Kihel, Badra and Chouraqui, Samira},
  journal={IEEE Open Journal of Vehicular Technology},
  title={TIG*: Enhanced Tangent Intersection Guidance for Efficient 3D UAV Path Planning in Complex Environments},
  year={2026},
  doi={10.1109/OJVT.2026.3659786}
}
```

---

## 🏛 Affiliation

H. Cheriet  
University of Science and Technology of Oran – Mohamed Boudiaf (USTO-MB)  
Algeria  

---

## 🔬 Research Extensions

Possible extensions include:

- AI-assisted tangent selection  
- Neural waypoint prediction  
- Hybrid TIG–sampling planners  
- Real-time embedded UAV implementation  
- Dynamic obstacle avoidance integration  

---

## 📄 License

This project is released for academic and research purposes.  
Users are requested to cite the associated IEEE publication when using this code.
