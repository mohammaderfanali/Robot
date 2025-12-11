<div align="center">
  <img src="https://cdn.freebiesupply.com/logos/large/2x/sharif-logo-png-transparent.png" width="150" height="150" alt="Sharif University Logo">
  <br><br>
  <h1 align="center">Robotics Practical Assignments: EKF Sensor Fusion</h1>
</div>

---

### :dart: About This Repository

This repository hosts a series of practical assignments for the **Robotics** course (Sharif University of Technology).

**Assignment 1** focuses on implementing an **Extended Kalman Filter (EKF)** from scratch in ROS 2 (Python) to achieve robust robot localization by fusing data from multiple sensors. The implementation follows the standard EKF prediction-correction loop. 

* **Objective:** Implement a custom EKF node to fuse Odometry (Wheel Encoders), Visual Odometry (VO), and IMU Yaw into a single, reliable state estimate.
* **State Vector:** $\mathbf{x} = [x, y, \theta]^T$ (Position and Yaw).
* **Environment:** ROS 2 (Jazzy/Humble) running on Ubuntu.

### :robot: Core Implementation: Extended Kalman Filter (EKF)

The project consists of three main custom ROS 2 nodes, designed to operate in parallel, enabling the data fusion process:

#### 1. :wheel: `prediction_node.py` (Motion Model)
* **Role:** Provides the *a priori* state estimate (Prediction Step).
* **Input:** Linear and angular velocities (Twist) from the wheel odometry source.
* **Function:** Implements the kinematic model of the differential drive robot to calculate state changes ($\Delta x, \Delta y, \Delta \theta$) and publishes the raw wheel odometry.
* **Output:** `Odometry` message on `/wheel_odom/odom` (Prediction).

#### 2. :satellite: `measurement_node.py` (Sensor Preprocessing)
* **Role:** Prepares the combined sensor reading for the EKF Correction Step.
* **Input:** Raw high-rate sensor readings, specifically the position components ($X, Y$) from Visual Odometry (VO) and the orientation component ($\theta$) from the IMU.
* **Function:** Creates a unified `Odometry` message containing the sensor measurements ($Z$) and, crucially, sets the **Measurement Noise Covariance Matrix ($\mathbf{R}$)** based on sensor uncertainties or defined parameters.
* **Output:** `Odometry` message on `/ekf/measurement` containing $X, Y, \theta$ and the $6 \times 6$ covariance matrix $\mathbf{R}$.

#### 3. :brain: `ekf_node.py` (Fusion Core)
* **Role:** Executes the main filtering logic, fusing the Prediction and the Measurement.
* **Function:** Implements the core EKF algorithm:
    * **Prediction Step:** Uses the Prediction input to calculate the predicted state ($\mathbf{x}_k^-$) and covariance ($\mathbf{P}_k^-$). This step relies on the **Process Noise Covariance ($\mathbf{Q}$)** matrix.
    * **Correction Step:** Uses the Measurement input to calculate the Kalman Gain ($\mathbf{K}$) and update the final filtered state ($\mathbf{x}_k$) and final covariance ($\mathbf{P}_k$).

* **Output:** Publishes the final, filtered `Odometry` message on the `/ekf/odom` topic (the robot's best position estimate).

### :chart_with_upwards_trend: Tuning and Analysis

The stability and accuracy of the EKF are entirely dependent on properly setting the $\mathbf{Q}$ and $\mathbf{R}$ covariance matrices. This project includes extensive debugging and tuning of these parameters.

| Matrix | Parameter Name | Function | Impact on Performance |
| :--- | :--- | :--- | :--- |
| $\mathbf{Q}$ | Process Noise (in `ekf_node`) | Uncertainty of the Motion Model (Wheels). | If **too small**, the EKF trusts the Prediction too much and will drift. |
| $\mathbf{R}$ | Measurement Noise (in `measurement_node`) | Uncertainty of the Sensor Measurements (VO/IMU). | If **too small**, the EKF trusts the Measurement too much and will jump/jitter (high noise). |

#### Analysis Tools
* **`ros2 bag record`:** Used to log key data topics (`/ekf/odom`, `/wheel_odom/odom`, `/ekf/measurement`).
* **Foxglove Studio / rqt_plot:** Used for visual comparison of time series data ($X, Y, \theta$) and 2D/3D robot paths to evaluate filtering performance.

### :wrench: Tools & Technologies Used

* **ROS 2 (Robot Operating System 2):** Core framework for node communication and data handling.
* **Python 3:** Primary language for EKF and support node implementation.
* **NumPy:** Essential for matrix operations, covariance management, and the core EKF mathematics.
* **`tf_transformations`:** Utilized for converting between quaternion and Euler (Yaw) representations.

### 👨‍💻 Author
* **Mohammadreza Monemian**
