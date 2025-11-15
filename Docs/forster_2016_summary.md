# Summary of Forster et al. (2016): On-Manifold Preintegration for Real-Time Visual–Inertial Odometry

## 1. Problem Overview
Visual–Inertial Odometry (VIO) estimates a platform's pose using a camera and an Inertial Measurement Unit (IMU). IMUs generate high-rate measurements (100–1000 Hz), while camera frames arrive far more slowly (10–60 Hz). Incorporating every IMU reading directly into an optimization back-end makes real-time inference computationally prohibitive.

**Goal:** Efficiently summarize all IMU data between camera frames into a *single* relative motion factor while preserving accuracy and correct manifold geometry.

## 2. Main Contributions
- A rigorous IMU preintegration theory carried out directly on the rotation manifold **SO(3)**.
- Closed-form expressions for preintegrated measurements, covariance propagation, and Jacobians.
- A bias-correctable formulation: optimization can adjust biases without re-integrating IMU data.
- Integration into a factor-graph VIO framework (e.g., GTSAM) using a structureless vision model.
- Demonstrated real-time performance with high accuracy on synthetic and real datasets.

## 3. State & Sensor Model
### State at time $k$:
- Rotation $R_k \in SO(3)$  
- Position $p_k \in \mathbb{R}^3$  
- Velocity $v_k \in \mathbb{R}^3$  
- Gyro bias $b^g_k$  
- Accel bias $b^a_k$  

### IMU measurement model
- Gyroscope:  
  $$
  \tilde{\omega}(t) = \omega(t) + b^g(t) + \eta^g
  $$
- Accelerometer:  
  $$
  \tilde{a}(t) = R(t)^T (a(t) - g) + b^a(t) + \eta^a
  $$

Biases follow random walk; noises are zero-mean white.

## 4. What is Preintegration?
**Purpose:** Replace hundreds of raw IMU readings between camera frames $i$ and $j$ with a single *relative* measurement consisting of:

- Relative rotation $\Delta R_{ij}$  
- Relative velocity change $\Delta v_{ij}$  
- Relative position change $\Delta p_{ij}$  

These are computed using only IMU data and the current bias estimate at frame $i$.

**IMU integration:** Computes the full trajectory by sequentially integrating raw IMU measurements from a fixed initial state — any change in the initial state or biases requires recomputing everything.

**IMU preintegration:** Computes the relative motion between keyframes once and encodes it with bias and noise corrections, so updates during optimization can be applied without redoing the full integration.

## 5. Why On-Manifold?
Rotations are **not vectors in Euclidean space**; they live on a curved space called the rotation manifold **SO(3)**. Treating them as simple 3D vectors leads to inconsistencies and errors (like gimbal lock or incorrect linearization).

Key points for the on-manifold approach:
- **Exponential map** ($\exp$) converts rotation vectors in the tangent space (angular velocity $\cdot$ dt) into a proper rotation on $SO(3)$.
- **Log map** ($\log$) converts a rotation difference back into the tangent space for residual computation.
- All perturbations and Jacobians are expressed in the tangent space, ensuring **accurate linearization**.

**Benefit:** By respecting manifold geometry, we integrate rotations, propagate uncertainty, and compute Jacobians without introducing approximation errors.

## 6. Preintegration Mathematics
### Rotation increment per IMU sample
$$
\delta R = \exp\!\left((\tilde{\omega} - b^g - \eta^g)\,\Delta t\right)
$$

### Velocity and position increments
Accelerometer readings are rotated using the current rotation estimate to accumulate:
- Integrated acceleration → $\Delta v_{ij}$
- Integrated velocity → $\Delta p_{ij}$

Noise terms and bias perturbations are propagated through first-order models.

## 7. Bias Correction
Preintegrated quantities depend on the bias at time $i$. During optimization, if the bias updates by $\delta b$, the preintegrated values can be corrected **without recomputing the integration**:

- Linear bias-correction terms are stored during integration.
- Greatly improves efficiency in iterative optimization.

## 8. Factor Graph Formulation
A single **IMU factor** connects states at frames $i$ and $j$. It encodes:

- Preintegrated mean $(\Delta R_{ij}, \Delta v_{ij}, \Delta p_{ij})$  
- Covariances  
- Jacobians w.r.t. $R_i, v_i, p_i, b^g_i, b^a_i$ and similarly at $j$

**Structureless visual factors** remove 3D landmarks analytically, leaving only pose/velocity/bias variables in the graph.

## 9. Algorithmic Notes
- Preintegrate IMU readings between keyframes.  
- Maintain covariance and Jacobian propagation during integration.  
- Apply bias correction during optimization.  
- Use analytic Jacobians for stability and speed.  
- Implemented using iSAM2 / GTSAM for incremental optimization.

## 10. Experimental Results
- Validated on synthetic sequences and real datasets (e.g., EuRoC MAV).  
- Achieves real-time performance.  
- More accurate or competitive with state-of-the-art VIO systems of the time.  
- Reduced computation due to compact factor graphs and preintegration.

## 11. Limitations
- Relies on first-order approximations; very long intervals or extreme motion may accumulate error.  
- Requires correct IMU noise modeling and time synchronization.  
- Keyframe rate influences numerical stability.

## 12. Impact
This work became the standard preintegration method adopted in:
- GTSAM (official implementation)  
- VIO systems such as VINS-Mono, OKVIS variants  
- Multi-sensor SLAM, event-based VIO, and modern tightly coupled estimators  

It is now foundational in visual–inertial estimation research.
