import serial
import struct
import numpy as np
import quaternion

DT = 4*1e-3

def skew(vec:np.array):
    x,y,z = vec
    return np.array([
        [0, -z, y],
        [z, 0, -x],
        [-y, x, 0]
    ])

class IMUReader:
    def __init__(self):
        self.ser = serial.Serial("/dev/ttyUSB0", 115200)

    def read(self): # Returns 6DOF IMU reading
        size = struct.calcsize("<6f")
        data = self.ser.read(size)
        imu_data = struct.unpack("<6f", data)
        return imu_data

class State:
    def __init__(self):
        self.q = quaternion.quaternion(1,0,0,0)
        self.ab = np.array([0,0,0], dtype="float").reshape((3,1))
        self.wb = np.array([0,0,0], dtype="float").reshape((3,1))

    def propagate(self, gyro:np.array, dt:float):
        rvec = ((gyro - self.wb))*dt
        self.q *= quaternion.from_rotation_vector(rvec.flatten())
    
    def update(self, err_state):
        self.q *= quaternion.from_rotation_vector(err_state.dtheta.flatten())
        self.ab += err_state.dab
        self.wb += err_state.dwb

class ErrorState:
    def __init__(self):
        self.statevect = np.zeros((9,1), dtype="float")
    
    @property 
    def dtheta(self):
        return self.statevect[:3, :]
    
    @dtheta.setter
    def dtheta(self, val):
        self.statevect[:3, :] = val

    @property 
    def dab(self):
        return self.statevect[3:6, :]
    
    @dab.setter 
    def dab(self, val):
        self.statevect[3:6, :] = val
    
    @property
    def dwb(self):
        return self.statevect[6:, :]
    
    @dwb.setter
    def dwb(self, val):
        self.statevect[6:, :] = val
    
    def propagate(self, state:State, gyro:np.array, dt:float):
        RT = quaternion.from_rotation_vector((gyro-state.wb).flatten()*dt).conjugate()
        self.dtheta = quaternion.rotate_vectors(RT, self.dtheta.flatten()).reshape((3,1)) - self.dwb*dt
    
    def reset(self):
        self.statevect = np.zeros((9,1), dtype="float")

class Eskf: 
    def __init__(self):
        self.state = State()
        self.error_state = ErrorState()    

        self.accel_var = np.square(0.005)                *np.eye(3,3)
        self.A_i =       1e-10                           *DT*np.eye(3,3)
        self.Theta_i = np.square(1/180*3.14159)          *DT*DT*np.eye(3,3)
        self.Omega_i = 1e-12                              *DT* np.eye(3,3)

        self.flat_var = np.square(5.0/180*3.14159)       *np.eye(3,3)

        self.F = np.eye(9,9)
        self.P = np.eye(9,9)
        self.Q = np.eye(9,9)
        self.H = None

        # Construct Q
        self.flat_var[2][2] /= 1e4
        self.Q[:3, :3] = self.Theta_i
        self.Q[3:6, 3:6] = self.A_i
        self.Q[6:9, 6:9] = self.Omega_i

        # Construct P
        self.P[:3, :3] = self.flat_var
        self.P[2][2] = 1e-6
        self.P[3:6, 3:6] = self.A_i * 1e-2
        self.P[6:9, 6:9] = self.Omega_i * 1e-2

    def propagate(self, gyro:np.array, dt:float):
        """
        gyro: 3x1 vector of gyroscope measurement
        """
        self.state.propagate(gyro, dt)       
        self.error_state.propagate(self.state, gyro, dt)

        # Constructing F
        self.F = np.eye(9,9)
        gyro_prop_q = quaternion.from_rotation_vector((gyro - self.state.wb).flatten() * dt)
        RT = quaternion.as_rotation_matrix(gyro_prop_q).T
        self.F[:3, :3] = RT.reshape((3,3))
        self.F[:3, 6:9] = -dt * np.eye(3,3)

        self.P = self.F.T @ self.P @ self.F + self.Q
        if self.P[2][2] > 1e-6:
            self.P[2][2] = 1e-6

    def update(self, accel:np.array):
        linear_accel_var = np.square(np.linalg.norm(accel) - 1)
        V = self.accel_var + linear_accel_var * np.eye(3,3)
        H = np.zeros((3,9), dtype="float")
        RT_q_g = quaternion.rotate_vectors(self.state.q.conjugate(), np.array([0,0,-1], dtype="float"))

        H[:3, :3] = skew(RT_q_g)
        H[:3, 3:6] = np.eye(3,3)

        w,x,y,z = self.state.q.w, self.state.q.x, self.state.q.y, self.state.q.z

        H_xq = np.array([ 
            [y, -z, w, -x],
            [-x, -w, -z, -y],
            [-w, x, y, -z]
        ])

        Q_dtheta = np.array([
            [-x, -y, -z],
            [w, -z, y],
            [z, w,-x],
            [-y, x, w]
        ])

        H_x = np.hstack((H_xq, np.eye(3,3), np.zeros((3,3))))
        X_dx = np.vstack([np.hstack([Q_dtheta, np.zeros((4,6))]), np.hstack([np.zeros((6,3)), np.eye(6,6)])])
        
        self.H = H_x @ X_dx 

        K = self.P @ self.H.T @ np.linalg.inv((self.H @ self.P @ self.H.T + V))
        self.error_state.statevect += K @ (accel - (RT_q_g.reshape(3,1) + self.state.ab))
        I = np.eye(9,9, dtype=np.float32)
        self.P = (I - K @ self.H) @ self.P @ (I - K @ self.H).T + K @ V @ K.T



    def inject(self):
        self.state.update(self.error_state)
        self.error_state.reset()


    

if __name__ == "__main__":
    q = quaternion.from_rotation_vector(np.array([1,1,1]).reshape((1,3)))
    rv = quaternion.rotate_vectors(q, np.array([0,0,1]))
    print(rv)
    