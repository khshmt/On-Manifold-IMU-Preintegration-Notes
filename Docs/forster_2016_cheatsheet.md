# On-Manifold Preintegration Cheat-Sheet (Equations & Jacobians)

This cheat-sheet summarizes the core mathematical equations and Jacobians required to implement the on-manifold IMU preintegration method from Forster et al. (2016).

---

# 1. Notation
- Rotation: $R \in SO(3)$  
- Exponential map: $\exp(\cdot) : \mathfrak{so}(3) \to SO(3)$  
- Log map: $\log(\cdot) : SO(3) \to \mathfrak{so}(3)$  
- Skew operator: $[v]_\times$  
- IMU measurements:
  - Gyro: $\tilde \omega = \omega + b^g + \eta^g$  
  - Accel: $\tilde a = R^T(a - g) + b^a + \eta^a$

---

# 2. Preintegrated Measurements
Preintegrated terms express relative motion between keyframes $i$ and $j$:

## 2.1 Preintegrated Rotation
$$
\Delta \tilde R_{ij} = \prod_{k=i}^{j-1} \exp\big((\tilde\omega_k - b_i^g)\,\Delta t\big)
$$

Noise Model:
$$
\delta \phi_{ij} \approx \sum F_R\,\eta^g
$$
where $\delta\phi$ is the rotation perturbation in the tangent space.

## 2.2 Preintegrated Velocity
$$
\Delta \tilde v_{ij} = \sum_{k=i}^{j-1} \Delta \tilde R_{ik}(\tilde a_k - b_i^a)\,\Delta t
$$
$$
\delta v = J_{v,\phi}\,\delta\phi + J_{v,ba}\,\delta b^a + J_{v,bg}\,\delta b^g + n_v
$$

## 2.3 Preintegrated Position
$$
\Delta \tilde p_{ij} = \sum_{k=i}^{j-1} \left[ \Delta \tilde v_{ik}\,\Delta t + \frac12 \Delta\tilde R_{ik}(\tilde a_k - b_i^a)\,\Delta t^2 \right]
$$
$$
\delta p = J_{p,\phi}\,\delta\phi + J_{p,ba}\,\delta b^a + J_{p,bg}\,\delta b^g + n_p
$$

---

# 3. Residuals (Factor Graph)
The IMU factor between states $x_i$ and $x_j$ has three residual blocks.

## 3.1 Rotation Residual
$$
r_R = \log\left( (\Delta \tilde R_{ij} \cdot \exp(J_{R,bg}\,\delta b^g))^T R_i^T R_j \right)
$$

## 3.2 Velocity Residual
$$
r_v = R_i^\top \left( v_j - v_i - g\,\Delta t_{ij} \right) 
      - \left( \Delta \tilde v_{ij} + J_{v,bg}\,\delta b^g + J_{v,ba}\,\delta b^a \right)
$$

## 3.3 Position Residual
$$
r_p = R_i^\top \left( p_j - p_i - v_i\,\Delta t_{ij} - \frac12 g\,\Delta t_{ij}^2 \right) 
      - \left( \Delta \tilde p_{ij} + J_{p,bg}\,\delta b^g + J_{p,ba}\,\delta b^a \right)
$$

---

# 4. Jacobians
The IMU factor connects variables:
$$
x = [R_i, p_i, v_i, b^g_i, b^a_i, R_j, p_j, v_j]^T
$$

## 4.1 Rotation Residual Jacobians
$$
\frac{\partial r_R}{\partial R_i} = -J_r \cdot \mathrm{Ad}_{R_j^T R_i}, \quad
\frac{\partial r_R}{\partial R_j} = J_r, \quad
\frac{\partial r_R}{\partial b^g} = J_r \, J_{R,bg}
$$

## 4.2 Velocity Residual Jacobians
$$
\frac{\partial r_v}{\partial R_i} = -R_i^T [v_j - v_i - g\Delta t]_{\times}, \quad
\frac{\partial r_v}{\partial v_i} = -R_i^T, \quad
\frac{\partial r_v}{\partial v_j} = R_i^T, \quad
\frac{\partial r_v}{\partial b^g} = -J_{v,bg}, \quad
\frac{\partial r_v}{\partial b^a} = -J_{v,ba}
$$

## 4.3 Position Residual Jacobians
$$
\frac{\partial r_p}{\partial R_i} = -R_i^T[p_j - p_i - v_i\Delta t - \frac12 g\Delta t^2]_{\times}, \quad
\frac{\partial r_p}{\partial p_i} = -R_i^T, \quad
\frac{\partial r_p}{\partial p_j} = R_i^T, \quad
\frac{\partial r_p}{\partial v_i} = -R_i^T, \quad
\frac{\partial r_p}{\partial b^g} = -J_{p,bg}, \quad
\frac{\partial r_p}{\partial b^a} = -J_{p,ba}
$$

---

# 5. Bias Correction Terms
$$
\begin{aligned}
\Delta R_{ij}(b) &\approx \Delta R_{ij}(b_0) \cdot \exp(J_{R,bg}\,\delta b^g) \\
\Delta v_{ij}(b) &\approx \Delta v_{ij}(b_0) + J_{v,bg}\,\delta b^g + J_{v,ba}\,\delta b^a \\
\Delta p_{ij}(b) &\approx \Delta p_{ij}(b_0) + J_{p,bg}\,\delta b^g + J_{p,ba}\,\delta b^a
\end{aligned}
$$

---

# 6. Covariance Propagation
$$
\Sigma_{k+1} = F_k\,\Sigma_k\,F_k^T + G_k\,Q\,G_k^T
$$

Where:  
- $F_k$ is the state-transition Jacobian  
- $G_k$ embeds IMU noise into the state  
- $Q$ is IMU noise covariance  

---

# 7. Summary of Required Components
- SO(3) exponential/log maps  
- Preintegrated increments: $\Delta R, \Delta v, \Delta p$  
- Jacobians wrt bias: $J_{R,bg}, J_{v,bg}, J_{v,ba}, J_{p,bg}, J_{p,ba}$  
- Covariance propagation across IMU samples  
- Factor residuals $(r_R, r_v, r_p)$  
- Complete Jacobians wrt all state variables  

This is everything required to build the GTSAM-compatible IMU factor from scratch.
