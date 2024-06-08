# 칼만 필터 클래스 정의
import numpy as np

class KalmanFilter:
    def __init__(self, state_dim, meas_dim):
        self.state_dim = state_dim
        self.meas_dim = meas_dim

        # 상태 추정치 초기화
        self.x = np.zeros((state_dim, 1))
        
        # 상태 추정 오차 공분산 행렬 초기화
        self.P = np.eye(state_dim)
        
        # 상태 전이 행렬 (State transition matrix)
        self.F = np.eye(state_dim)
        
        # 측정 행렬 (Measurement matrix)
        self.H = np.zeros((meas_dim, state_dim))
        
        # 측정 노이즈 공분산 행렬 (Measurement noise covariance)
        self.R = np.eye(meas_dim)
        
        # 프로세스 노이즈 공분산 행렬 (Process noise covariance)
        self.Q = np.eye(state_dim)

    def predict(self):
        # 상태 예측
        self.x = np.dot(self.F, self.x)
        
        # 오차 공분산 예측
        self.P = np.dot(self.F, np.dot(self.P, self.F.T)) + self.Q

    def update(self, z):
        # 측정 잔차
        y = z - np.dot(self.H, self.x)
        
        # 잔차 공분산
        S = np.dot(self.H, np.dot(self.P, self.H.T)) + self.R
        
        # 칼만 이득
        K = np.dot(self.P, np.dot(self.H.T, np.linalg.inv(S)))
        
        # 상태 업데이트
        self.x = self.x + np.dot(K, y)
        
        # 오차 공분산 업데이트
        self.P = self.P - np.dot(K, np.dot(self.H, self.P))

    def get_state(self):
        return self.x
