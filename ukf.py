import numpy as np
from filterpy.kalman import UnscentedKalmanFilter as UKF
from filterpy.kalman import MerweScaledSigmaPoints

class UnscentedKalmanFilter:
    def __init__(self, dt, process_noise, measurement_noise):
        self.dt = dt  # 시간 간격
        
        # 상태 벡터: 위치 (x, y, z) 및 속도 (vx, vy, vz)
        self.state_dim = 6
        self.measurement_dim = 3  # GPS 측정 차원: 위치 (x, y, z)
        
        # 시그마 포인트 설정
        self.sigma_points = MerweScaledSigmaPoints(
            self.state_dim, alpha=0.1, beta=2., kappa=0.)
        
        # UKF 객체 생성
        self.ukf = UKF(dt=dt, dim_x=self.state_dim, dim_z=self.measurement_dim,
                       fx=self.state_transition_function,
                       hx=self.measurement_function,
                       points=self.sigma_points)
        
        # 초기 상태 벡터
        self.ukf.x = np.zeros(self.state_dim)
        
        # 공분산 행렬 설정
        self.ukf.P = np.eye(self.state_dim) * 0.1  # 초기 상태 공분산
        
        # 과정 노이즈 공분산 행렬
        self.ukf.Q = process_noise
        
        # 측정 노이즈 공분산 행렬 (GPS)
        self.ukf.R = measurement_noise

        # 위치 변화의 최대 허용 범위
        self.max_position_change = 5
    
    def state_transition_function(self, x, dt):
        """ 상태 전이 함수: IMU 데이터를 이용해 상태를 예측 """
        F = np.array([[1, 0, 0, dt,  0,  0],
                      [0, 1, 0,  0, dt,  0],
                      [0, 0, 1,  0,  0, dt],
                      [0, 0, 0,  1,  0,  0],
                      [0, 0, 0,  0,  1,  0],
                      [0, 0, 0,  0,  0,  1]])
        return np.dot(F, x)
    
    def measurement_function(self, x):
        """ 측정 함수: GPS 데이터를 이용해 상태를 관측 """
        return x[:3]  # 위치 (x, y, z)만 반환
    
    def predict(self, imu_accel):
        """ 상태 예측 """
        # IMU 가속도를 속도 상태로 통합
        self.ukf.x[3:] += imu_accel * self.dt
        self.ukf.predict(dt=self.dt)
    
    def update(self, gps_data, imu_accel):
        previous_position = self.ukf.x[:3].copy()  # 이전 위치
        
        # GPS 데이터를 사용하여 상태 갱신
        self.ukf.update(gps_data)
        
        # # 예측된 새로운 위치
        # new_position = self.ukf.x[:3]
        
        # # 이전 위치와 새로운 위치의 차이 계산
        # position_change = np.linalg.norm(new_position - previous_position)
        # print(position_change)
        #   # IMU 데이터를 이용해 방향 벡터 계산
        # imu_direction = imu_accel / np.linalg.norm(imu_accel) if np.linalg.norm(imu_accel) != 0 else np.zeros(3)
        
        # # 위치 변화가 허용 범위를 초과하는지 확인
        # if position_change > self.max_position_change:
        #     print(f"경고: 위치 변화 {position_change}가 허용 범위 {self.max_position_change}를 초과합니다.")
        #     # 방향 벡터를 사용하여 새로운 위치를 임계값 내로 제한
        #     self.ukf.x[:3] = previous_position + imu_direction
    
    def get_state(self):
        """ 현재 상태 반환 """
        return self.ukf.x