from .kalman_filter import KalmanFilter
import numpy as np


class CartKalmanFilter(KalmanFilter):
    def __init__(self, X, dt=0.002, x_cov=9e-3, dx_cov=0, obs_noise_cov=0.0012):
        # Time interval
        self.sz = X.shape[0]

        # State vector
        self.X = np.append(X, np.zeros((self.sz,)))

        # Motion Model
        self.F = np.diag(
            np.ones(
                2 * self.sz,
            )
        )
        self.F[: self.sz, self.sz :] = np.diag(np.full((self.sz,), dt))

        # Motion Noise Covariance
        self.Q = np.diag(
            np.concatenate([np.full((self.sz,), x_cov), np.full((self.sz,), dx_cov)])
        )

        # Correlation Matrix
        self.P = self.Q

        # Observation Model
        self.H = np.zeros((self.sz, 2 * self.sz))
        np.fill_diagonal(self.H, 1)

        # Observation Noise Covariance (load - grav)
        self.R = np.diag(np.full((self.sz,), obs_noise_cov))

        self.S = np.zeros((self.sz, self.sz))
        self.K = self.X