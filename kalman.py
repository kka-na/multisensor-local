import numpy as np

class KalmanFilter:
    def __init__(self):
        self.state = np.zeros(9)  # [x, y, z, vx, vy, vz, ax, ay, az]
        self.covariance = np.eye(9) * 0.1
        self.process_noise = np.eye(9) * 0.1
        self.measurement_noise = np.eye(3) * 0.5  # Measurement noise for position only
        self.measurement_matrix = np.zeros((3, 9))  # Measurement matrix for position only
        self.measurement_matrix[0, 0] = 1
        self.measurement_matrix[1, 1] = 1
        self.measurement_matrix[2, 2] = 1
        
    def predict(self, dt):
        # State transition matrix considering constant acceleration model
        F = np.eye(9)
        F[0, 3] = dt
        F[1, 4] = dt
        F[2, 5] = dt
        F[3, 6] = dt
        F[4, 7] = dt
        F[5, 8] = dt
        
        # Control matrix (for acceleration input)
        B = np.zeros((9, 3))
        B[3, 0] = dt
        B[4, 1] = dt
        B[5, 2] = dt
        
        # Predicted state
        self.state = np.dot(F, self.state)
        
        # Predicted covariance
        self.covariance = np.dot(F, np.dot(self.covariance, F.T)) + self.process_noise
        
    def update(self, measurement):
        # Measurement residual
        y = measurement - np.dot(self.measurement_matrix, self.state)
        
        # Residual covariance
        S = np.dot(self.measurement_matrix, np.dot(self.covariance, self.measurement_matrix.T)) + self.measurement_noise
        
        # Kalman gain
        K = np.dot(self.covariance, np.dot(self.measurement_matrix.T, np.linalg.inv(S)))
        
        # Update state
        self.state += np.dot(K, y)
        
        # Update covariance
        self.covariance = np.dot(np.eye(9) - np.dot(K, self.measurement_matrix), self.covariance)

        
    def integrate_imu(self, accel, dt):
        # Integrate acceleration to get velocity and then position
        self.state[3:6] += accel * dt  # Update velocity (vx, vy, vz)
        self.state[0:3] += self.state[3:6] * dt  # Update position (x, y, z)
