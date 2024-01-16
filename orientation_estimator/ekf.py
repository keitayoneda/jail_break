import numpy as np

class EKF:
    def __init__(self, init_x, init_cov,  A_func, C_func, pred_func, update_cov, obs_cov):
        self.A_func = A_func
        self.C_func = C_func
        self.pred_func = pred_func
        self.update_cov = update_cov
        self.obs_cov = obs_cov
        self.cov = init_cov
        self.x = init_x

    def update(self):
        A = self.A_func()
        self.x = A @ self.x
        self.cov = A @ self.cov @ A.T + self.update_cov

    def observe(self, y):
        C = self.C_func(self.x)
        Sn = C @ self.cov @ C.T + self.obs_cov
        delta_y = y - self.pred_func(self.x)
        kalman_gain = self.cov @ C.T @ np.linalg.inv(Sn + C @ self.cov @ C.T)

        self.x = self.x + kalman_gain @ delta_y
        self.cov = self.cov  - kalman_gain @ C @ self.cov

    def normalizeX(self):
        self.x = self.x / np.linalg.norm(self.x)

