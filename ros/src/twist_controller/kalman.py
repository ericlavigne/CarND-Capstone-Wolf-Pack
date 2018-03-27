from math import atan2, pi, sqrt
import numpy as np
from numpy.linalg import cholesky, inv

def sigma_points(x,P,lambda_n):
    nx = len(x)
    lamb = lambda_n - nx
    #print("P:")
    #print(P)
    A = sqrt(lambda_n) * cholesky(P)
    #print("A:")
    #print(A)
    res = np.zeros((nx, 2 * nx + 1))
    res[:,0] = x
    for i in range(nx):
        res[:,1+i] = x + A[:,i]
        res[:,1+i+nx] = x - A[:,i]
    #print("sigma res:")
    #print(res)
    return res

class Kalman(object):

    def __init__(self, state, stdev):
        self.nx = len(state)
        nstd = len(stdev)
        assert self.nx == nstd, "state %r and stdev %r should have the same length" % (self.nx, nstd)
        self.x = np.zeros(self.nx)
        for i in range(self.nx):
            self.x[i] = state[i]
        self.P = np.zeros((self.nx,self.nx))
        for i in range(self.nx):
            self.P[i,i] = stdev[i]
        self.lambda_n = 3.0 # lambda = lambda_n - n

    # predict_fn(input_sigma_point_with_noise, output_sigma_point_without_noise)
    def predict(self, noise_stdev, predict_fn):
        nx = self.nx
        nn = len(noise_stdev)
        naux = nx + nn
        nsigma = 2 * naux + 1

        x_aug = np.zeros(naux)
        x_aug[0:nx] = self.x

        P_aug = np.zeros((naux,naux))
        P_aug[0:nx,0:nx] = self.P
        for i in range(nn):
            P_aug[nx+i, nx+i] = noise_stdev[i] * noise_stdev[i]

        sigma_old = sigma_points(x_aug, P_aug, self.lambda_n)
        sigma_new = np.zeros((nx, nsigma))
        for i in range(nsigma):
            predict_fn(sigma_old[:,i], sigma_new[:,i])

        lambda_ = self.lambda_n - naux
        weights = np.zeros(nsigma)
        weights.fill(0.5 / self.lambda_n)
        weights[0] = lambda_ / self.lambda_n

        self.x.fill(0.0)
        for i in range(nsigma):
            self.x += weights[i] * sigma_new[:,i]

        self.P.fill(0.0)
        for i in range(nsigma):
            diff = sigma_new[:,i] - self.x
            self.P += weights[i] * np.outer(diff, diff)

        # Ensure numerical stability of the new P value
        self.regularize()

    # measure_fn(input_sigma_point, output_expected_measurement)
    def measure(self, measure_data, measure_stdev, measure_fn):
        nz = len(measure_data)
        z = np.zeros(nz)
        stdz = np.zeros(nz)
        for i in range(nz):
            z[i] = measure_data[i]
            stdz[i] = measure_stdev[i]

        nsigma = 2 * self.nx + 1
        sigma_state = sigma_points(self.x, self.P, self.lambda_n)
        sigma_measure = np.zeros((nz, nsigma))
        for i in range(nsigma):
            measure_fn(sigma_state[:,i], sigma_measure[:,i])

        lambda_ = self.lambda_n - self.nx
        weights = np.zeros(nsigma)
        weights.fill(0.5 / self.lambda_n)
        weights[0] = lambda_ / self.lambda_n

        # mean of predicted measurement
        z_pred = np.zeros(nz)
        for i in range(nsigma):
            z_pred += weights[i] * sigma_measure[:,i]

        # measurement covariance
        S = np.zeros((nz,nz))
        for i in range(nsigma):
            diff = sigma_measure[:,i] - z_pred
            S += weights[i] * np.outer(diff, diff)
        for i in range(nz):
            S[i] += measure_stdev[i] * measure_stdev[i]

        # cross-correlation
        Tc = np.zeros((self.nx, nz))
        for i in range(nsigma):
            z_diff = sigma_measure[:,i] - z_pred
            x_diff = sigma_state[:,i] - self.x
            Tc += weights[i] * np.outer(x_diff, z_diff)

        # kalman gain
        K = np.dot(Tc, inv(S))

        # update state mean and covariance
        z_diff = z - z_pred
        x_diff = np.dot(K, z_diff)
        self.x += x_diff
        self.P -= np.dot(np.dot(K, S), K.transpose())

        # Ensure numerical stability of the new P value
        self.regularize()

    def regularize(self):
        # Ensure numerical stability via some hints from a StackExchange answer
        # https://robotics.stackexchange.com/questions/2000/maintaining-positive-definite-property-for-covariance-in-an-unscented-kalman-fil

        # Covariance matrix should be symmetric P[i,j] = P[j,i]
        # with each i,j/j,i pair containing the covariance between features i and j
        self.P = 0.5 * (self.P + self.P.transpose())

        # Diagonal values should be positive
        # representing the individual uncertainties of each feature
        for i in range(self.P.shape[0]):
            if self.P[i,i] < 0.001:
                self.P[i,i] = 0.001
