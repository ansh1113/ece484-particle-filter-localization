# Monte Carlo Localization for Indoor Robot Navigation

[![Python](https://img.shields.io/badge/Python-3.8+-blue.svg)](https://www.python.org/)
[![ROS2](https://img.shields.io/badge/ROS2-Humble-blue.svg)](https://docs.ros.org/en/humble/)
[![License](https://img.shields.io/badge/License-MIT-green.svg)](LICENSE)
[![Gazebo](https://img.shields.io/badge/Gazebo-11-orange.svg)](https://gazebosim.org/)

**Particle filter-based robot localization using LiDAR in GPS-denied environments - UIUC ECE 484 Safe Autonomy**

## 🎯 Key Results

- ✅ **Sub-meter Accuracy** - Average localization error < 0.5m after convergence
- ✅ **Fast Convergence** - Particles converge within 50-100 iterations
- ✅ **Real-time Performance** - Runs at 10 Hz with 1500 particles
- ✅ **8-Direction Sensing** - Enhanced LiDAR processing for improved accuracy
- ✅ **GPS-Denied Operation** - Fully autonomous indoor localization

---

## 📋 Table of Contents

- [Overview](#overview)
- [System Architecture](#system-architecture)
- [Algorithm Details](#algorithm-details)
- [Installation](#installation)
- [Usage](#usage)
- [Experimental Results](#experimental-results)
- [Performance Analysis](#performance-analysis)
- [Course Context](#course-context)
- [Team](#team)

---

## Overview

This project implements **Monte Carlo Localization (MCL)** - a particle filter-based probabilistic algorithm for robot localization. The system localizes a mobile robot navigating the first floor of ECEB at UIUC using LiDAR measurements and a known map, without GPS.

### Problem Statement

**Given:**
- Detailed 2D occupancy grid map of ECEB
- LiDAR sensor measurements (8 directions)
- Odometry from robot control inputs

**Find:**
- Robot's position (x, y) and orientation (θ) in real-time

### Why Particle Filters?

- ✅ **Non-parametric** - Represents multi-modal distributions
- ✅ **Robust** - Handles non-Gaussian noise and non-linear dynamics
- ✅ **Efficient** - Parallelizable and scalable
- ✅ **Versatile** - Works with any sensor model

---

## 🏗️ System Architecture
```
┌─────────────────┐
│ Gazebo Simulator│
│   (ECEB Map)    │
└────────┬────────┘
         │ Raw Point Cloud
         ▼
┌─────────────────┐      ┌──────────────┐
│ LiDAR Processing│─────▶│ Sensor Model │
│  (8 directions) │      │  (Particles) │
└─────────────────┘      └──────┬───────┘
         │                      │
         │ Measurements         │ Predicted
         │                      │ Measurements
         ▼                      ▼
┌──────────────────────────────────┐
│   Monte Carlo Localization       │
│  ┌────────────────────────────┐  │
│  │ 1. Motion Update           │  │
│  │ 2. Measurement Update      │  │
│  │ 3. Weight Calculation      │  │
│  │ 4. Resampling              │  │
│  └────────────────────────────┘  │
└───────────────┬──────────────────┘
                │
                ▼
         ┌──────────────┐
         │  Estimated    │
         │  Pose (x,y,θ) │
         └──────────────┘
```

### Components

**1. LiDAR Processing (`lidar_processing.py`)**
- Subscribes to raw point cloud from Gazebo
- Processes 360° scan into 8 directional measurements
- Directions: Front (0°), Front-Right (45°), Right (90°), Rear-Right (135°), Rear (180°), Rear-Left (225°), Left (270°), Front-Left (315°)
- Returns distances to nearest obstacles in each direction

**2. Sensor Model (`maze.py`)**
- Ray-casting on 2D occupancy grid map
- Simulates LiDAR measurements for each particle
- Sensor range limit: 15m (configurable)

**3. Particle Filter (`particle_filter.py`)**
- Maintains distribution of 250-1500 particles representing possible robot poses
- Implements MCL algorithm with motion and measurement updates
- Adaptive resampling with GPS integration for bonus problem

**4. Vehicle Controller (`vehicle.py`)**
- Provides odometry/control signals
- Follows pre-defined waypoints through ECEB

---

## 🔬 Algorithm Details

### Monte Carlo Localization Algorithm

#### 1. **Initialization**
```python
# Uniformly distribute N particles in free space
particles = []
for i in range(N):
    x = random(0, map_width)
    y = random(0, map_height)
    theta = random(0, 2π)
    weight = 1/N
    particles.append(Particle(x, y, theta, weight))
```

#### 2. **Motion Update (Prediction)**

Given control input `u = [v, δ]` (velocity, steering angle), propagate each particle using vehicle dynamics:
```python
# Vehicle dynamics model
dx/dt = v * cos(θ)
dy/dt = v * sin(θ)
dθ/dt = δ

# Integrate using scipy.integrate.ode with dt = 0.01s
# Add motion noise for diversity
x_new += random_normal(0, σ_x=0.02)
y_new += random_normal(0, σ_y=0.02)
θ_new += random_normal(0, σ_θ=0.9)
```

#### 3. **Measurement Update (Correction)**

Calculate particle weights using Gaussian likelihood:
```python
for particle in particles:
    # Get predicted LiDAR readings for this particle
    z_predicted = sensor_model(particle.pose)
    
    # Compare with actual sensor reading z_actual
    squared_diff = sum((z_actual[i] - z_predicted[i])^2)
    
    # Gaussian weight with σ = 2.0
    weight = exp(-squared_diff / (2 * σ^2))
    
    particle.weight = weight

# Normalize weights
total = sum(p.weight for p in particles)
for p in particles:
    p.weight /= total
```

#### 4. **Resampling**

Multinomial resampling with GPS integration (bonus):
```python
# 70% particles resampled based on weights
cumulative_weights = cumsum([p.weight for p in particles])

new_particles = []
for i in range(0.7 * N):
    r = random(0, 1)
    idx = bisect.bisect_left(cumulative_weights, r)
    new_particle = particles[idx].copy_with_noise()
    new_particles.append(new_particle)

# 30% random particles (80% GPS-guided if available)
for i in range(0.3 * N):
    if gps_available and random() < 0.8:
        # GPS-guided particle
        new_particle = Particle(
            x=gps_x + random_normal(0, 0.02),
            y=gps_y + random_normal(0, 0.02),
            theta=gps_heading + random_normal(0, 0.0005)
        )
    else:
        # Fully random particle
        new_particle = Particle(random_position())
    new_particles.append(new_particle)

particles = new_particles
```

#### 5. **Pose Estimation**
```python
# Weighted average for position
x_est = sum(p.x * p.weight for p in particles)
y_est = sum(p.y * p.weight for p in particles)

# Circular mean for heading
heading_sin = sum(sin(p.θ) * p.weight for p in particles)
heading_cos = sum(cos(p.θ) * p.weight for p in particles)
θ_est = atan2(heading_sin, heading_cos)
```

---

## 🚀 Installation

### Prerequisites
```bash
# System requirements
Ubuntu 22.04
ROS2 Humble
Gazebo 11
Python 3.8+
```

### ROS2 Package Dependencies
```bash
sudo apt install ros-humble-ros-control \
                 ros-humble-effort-controllers \
                 ros-humble-joint-state-controller \
                 ros-humble-ackermann-msgs
```

### Python Dependencies
```bash
pip install -r requirements.txt
```

**requirements.txt:**
```
numpy>=1.21.0
scipy>=1.7.0
matplotlib>=3.4.0
```

### Build Instructions
```bash
# Clone repository into ROS2 workspace
cd ~/ros2_ws/src
git clone https://github.com/ansh1113/ece484-particle-filter-localization.git mp3

# Build workspace
cd ~/ros2_ws
colcon build --symlink-install

# Source workspace
source install/setup.bash
```

---

## 💻 Usage

### Quick Start

**Terminal 1: Launch Gazebo Simulation**
```bash
source ~/ros2_ws/install/setup.bash
ros2 launch mp3 gem_vehicle.launch.py
```

**Terminal 2: Start Vehicle Controller**
```bash
cd ~/ros2_ws/src/mp3/src
python3 vehicle.py
```

**Terminal 3: Run Particle Filter**
```bash
cd ~/ros2_ws/src/mp3/src
python3 main.py --particles 500 --sensor_limit 15 --extensive_lidar
```

### Command Line Arguments
```bash
python3 main.py --help

Options:
  --particles N          Number of particles (default: 500)
  --sensor_limit D       Max sensor range in meters (default: 15)
  --show_frequency F     Visualization update rate (default: 10)
  --extensive_lidar      Use 8 directions instead of 4
  --gps_x_std σ         GPS noise std dev in x (default: 0)
  --gps_y_std σ         GPS noise std dev in y (default: 0)
  --gps_heading_std σ   GPS heading noise std dev (default: 0)
  --gps_update N        GPS update frequency Hz (default: 0, disabled)
```

### Setting Custom Start Position
```bash
# Reset vehicle to custom pose
python3 set_pos.py --x -45 --y 40 --heading 0

# Then run alternate waypoint sequence
python3 vehicle.py --alternate
```

### Example Configurations

**Standard Configuration (250 particles):**
```bash
python3 main.py --particles 250 --sensor_limit 15 --extensive_lidar
```

**High Accuracy (1500 particles):**
```bash
python3 main.py --particles 1500 --sensor_limit 20 --extensive_lidar
```

**With GPS Noise (Bonus Problem):**
```bash
python3 main.py --particles 1500 \
                --sensor_limit 15 \
                --gps_x_std 5 \
                --gps_y_std 5 \
                --gps_heading_std 0.393 \
                --gps_update 1 \
                --extensive_lidar
```

---

## 📊 Experimental Results

### Experiment 1: Effect of Particle Count

| Particles | Convergence Speed | Final Accuracy | Notes |
|-----------|------------------|----------------|-------|
| 250       | Moderate         | Good           | Best approximation with fewer particles |
| 750       | Fast             | Better         | Practical balance for real-time |
| 1500      | Fastest          | Best           | Highest accuracy, more stable |

**Key Finding**: Estimation accuracy improves with more particles following ~1/sqrt(N) relationship. 1500 particles provide best accuracy with stable convergence, while 750 offers practical balance between accuracy and computation.

### Experiment 2: Effect of Sensor Range

| Sensor Limit (m) | Accuracy | Convergence | Notes |
|------------------|----------|-------------|-------|
| 15               | Good     | Moderate    | Baseline configuration |
| 20               | Better   | Faster      | More informative features |
| 25               | Best     | Fastest     | Reduced ambiguity in open spaces |

**Key Finding**: Larger sensor limits increase accuracy when informative features are within range. However, excessively large limits may increase error if measurements hit max range frequently without detecting walls.

### Experiment 3: 4 vs 8 Measurement Directions

| Directions | Convergence Speed | Accuracy | Computational Cost |
|------------|------------------|----------|--------------------|
| 4          | Slower           | Good     | 1.0x (baseline)    |
| 8          | Faster           | Better   | 1.8x               |

**Key Finding**: 8 directional measurements significantly improve both convergence speed and estimation accuracy. The additional computational cost (1.8x) is justified by the performance gains.

### Experiment 4: Different Starting Positions

**Default Start (x=0, y=-98):**
- Normal convergence behavior
- Particles spread and converge smoothly

**Alternative Start (x=-45, y=40):**
- Slower convergence observed
- **Root Cause**: Vehicle collides with wall and gets stuck
- **Effect**: Motion model moves particles, but real position doesn't change (slip effect)
- **Lesson**: Physical constraints significantly impact filter performance

### Experiment 5: Measurement Noise (Bonus)

**Configuration**: 50% LiDAR dropout rate

**Observations**:
- Convergence slowed significantly
- Temporarily increased weight for incorrect particle states
- System eventually self-corrects but requires more iterations
- Highlights importance of reliable sensor data

### Performance by Environment Region

| Region        | Avg Error | Notes |
|---------------|-----------|-------|
| Corridors     | Low       | Strong geometric constraints |
| Intersections | Medium    | Multiple valid hypotheses |
| Open Spaces   | Higher    | Fewer distinctive features |
| Near Landmarks| Lowest    | Unique identifiable features |

**Key Finding**: Particle filter performs unevenly across environments. Regions with distinctive landmarks (corridors, corners) provide stronger measurement updates and lower prediction error.

---

## 🎥 Demo Videos

**Video Links**: [Google Drive - MP3 Demos](https://drive.google.com/drive/folders/1tji1z8HDMwo6BdeU8uxJp2NpQ15aoZ15)

### Visualization

The turtle graphics window shows:
- 🟢 **Green turtle**: Ground truth robot position
- 🟡 **Yellow turtle**: Estimated position (weighted particle mean)
- 🔵 **Blue arrows**: Individual particles (pose + heading + weight)
- ⬛ **Black**: Walls/obstacles  
- ⬜ **White**: Free space

**Particle color intensity** indicates weight (darker = higher weight)

---

## 📚 Course Context

**Course**: ECE 484 - Principles of Safe Autonomy  
**Institution**: University of Illinois Urbana-Champaign  
**Semester**: Fall 2025  
**Project Type**: Machine Problem 3 (Group Project)  
**Team**: Autoshield

---

## 👥 Team

**Team Autoshield**
- Ansh Bhansali - anshb3@illinois.edu
- Het Patel - hcp4@illinois.edu
- Sunny Deshpande - sunnynd2@illinois.edu
- Keisuke Ogawa - ogawa3@illinois.edu

---

## 📁 Project Structure
```
ece484-particle-filter-localization/
├── particle_filter.py       # Main MCL implementation
├── lidar_processing.py      # 8-direction LiDAR processing
├── maze.py                  # Map, Particle, Robot classes
├── main.py                  # Entry point with argument parsing
├── vehicle.py               # Vehicle controller
├── controller.py            # Pure pursuit lateral control
├── set_pos.py              # Utility to set vehicle position
├── requirements.txt         # Python dependencies
├── README.md               # This file
└── launch/
    └── gem_vehicle.launch.py  # ROS2 launch file
```

---

## 🛠️ Technical Implementation Highlights

### Sensor Model (8-Direction Ray-Casting)
```python
def sensor_model(coordinates, sensor_limit, orientation):
    """
    Ray-cast in 8 directions to measure distances
    
    Directions:
    - 0°: Front
    - 45°: Front-Right
    - 90°: Right
    - 135°: Rear-Right
    - 180°: Rear
    - 225°: Rear-Left
    - 270°: Left
    - 315°: Front-Left
    """
    directions = [0, π/4, π/2, 3π/4, π, 5π/4, 3π/2, 7π/4]
    distances = []
    
    for angle in directions:
        distance = 0
        dx = cos(orientation + angle)
        dy = sin(orientation + angle)
        
        while not collide_wall(x, y) and distance < sensor_limit:
            x += dx
            y += dy
            distance += 1
        
        distances.append(distance * 100)  # Convert to cm
    
    return distances
```

### GPS Integration (Bonus Problem)

To prevent particle depletion and improve robustness, we implemented a hybrid resampling strategy:

**70% Weighted Resampling** - Based on particle weights  
**30% Random Particles**:
- 80% GPS-guided (when available)
- 20% Fully random (exploration)

This prevents convergence to local minima and maintains particle diversity.

---

## 📖 Academic Integrity Statement

This repository contains coursework from ECE 484 - Principles of Safe Autonomy at UIUC.  
Shared for portfolio and educational purposes after course completion.

**If you are currently enrolled in this course:**
- ❌ Do NOT copy this code for your assignments
- ✅ Use only as a learning reference
- ✅ Follow your course's academic integrity policy

Violations of academic integrity policies will be reported.

---

## 📝 License

MIT License - See [LICENSE](LICENSE) for details

---

## 🙏 Acknowledgments

- ECE 484 course staff for infrastructure and guidance
- UIUC Robotics Lab for Gazebo ECEB environment
- Team Autoshield members for collaboration

---

## 📞 Contact

For questions about this implementation:
- **Ansh Bhansali**: anshbhansali5@gmail.com
- **GitHub**: [@ansh1113](https://github.com/ansh1113)

---

**⭐ If you find this helpful, please star the repository!**
