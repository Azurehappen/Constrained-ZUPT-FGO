# Constrained-Zero-Velocity-FGO

This repository contains the implementation of a constrained Factor Graph Optimization (FGO)-based and Extended Kalman Filter (EKF)-based pedestrian navigation system. The project supports trajectory estimation using foot-mounted IMUs with kinematic constraints, including equality constraints of Zero-Velocity Updates (ZUPTs) and inequality constraints of inter-foot distance limits.

## 📁 Project Structure

```
ekfZupt/             # MATLAB EKF implementation
zeroVelFgo/          # Python FGO implementation
figures/             # Experimental figures used in the paper
results/             # Error statistics and trajectory output
requirements.txt     # Python dependencies
```

## 📄 Citation

If you are interested in this work, please cite our paper:
```
@inproceedings{hu2025constrained,
  author    = {Yingjie Hu and Wang Hu},
  title     = {Constrained Factor Graph Optimization for Robust Networked Pedestrian Inertial Navigation},
  booktitle = {Proceedings of the IEEE/ION Position, Location and Navigation Symposium (PLANS)},
  year      = {2025},
  organization = {IEEE/ION}
}
```

## 📦 Requirements

We recommend using a conda environment on **Ubuntu**. To set up:

```
conda create -n fgo_nav python=3.9
conda activate fgo_nav
pip install -r requirements.txt
```

**Note**: GTSAM is included in the `requirements.txt`.

## 🚀 How to Run

### EKF-Based Approach

The EKF implementation is located in MATLAB. Run `generateEKFResults.m` for EKF results.


### FGO-Based Approach

The FGO approach is implemented in Python:

```
python runZuptFgo.py
```

By default, this script reads precomputed FGO results from saved `.csv` files. To rerun the optimization using GTSAM, set `read_zupt_xxx_from_csv = False` inside `runZuptFgo.py`.

