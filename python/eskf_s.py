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
        self.ser = serial.Serial("/dev/ttyUSB1", 115200)

    def read(self): # Returns 6DOF IMU reading
        size = struct.calcsize("<6f")
        data = self.ser.read(size)
        imu_data = struct.unpack("<6f", data)
        return imu_data

class State:
    def __init__(self):
        self.q = quaternion.from_rotation_vector(np.array([0,0,0]))

    def propagate(self, gyro:np.array, dt:float):
        rvec = gyro*dt
        self.q *= quaternion.from_rotation_vector(rvec.flatten())
    
    def update(self, err_state):
        self.q = self.q * quaternion.from_rotation_vector(err_state.dtheta.flatten())

class ErrorState:
    def __init__(self):
        self.statevect = np.zeros((3,1), dtype=np.float32)
    
    @property 
    def dtheta(self):
        return self.statevect[:3, :]
    
    @dtheta.setter
    def dtheta(self, val):
        self.statevect[:3, :] = val
    
    def propagate(self, state:State, gyro:np.array, dt:float):
        RT = quaternion.from_rotation_vector(gyro.flatten()*dt).conjugate()
        self.dtheta = quaternion.rotate_vectors(RT, self.dtheta.flatten()).reshape((3,1))
    
    def reset(self):
        self.statevect = np.zeros((3,1), dtype=np.float32)

class Eskf: 
    def __init__(self):
        self.state = State()
        self.error_state = ErrorState()    

        self.accel_var = np.square(0.005)                 *np.eye(3,3)
        self.Theta_i = np.square(1/180*3.141)          *DT*DT*np.eye(3,3)

        self.flat_var = np.square(5.0/180*3.14159)       *np.eye(3,3)

        self.F = np.eye(3,3)
        self.P = np.eye(3,3)
        self.Q = np.eye(3,3)
        self.H = None

        self.debug_froben_norm = 0
        self.max_index = None
        #self.Ra = np.eye(9,9) * self.accel_var

        # Construct Q
        self.flat_var[2][2] /= 1e4
        self.Q[:3, :3] = self.Theta_i

        # Construct P
        self.P[:3, :3] = self.flat_var

    def propagate(self, gyro:np.array, dt:float):
        """
        gyro: 3x1 vector of gyroscope measurement
        """
        self.state.propagate(gyro, dt)       
        self.error_state.propagate(self.state, gyro, dt)

        # Constructing F
        self.F = np.eye(3,3)
        gyro_prop_q = quaternion.from_rotation_vector(gyro.flatten() * dt)
        RT = quaternion.as_rotation_matrix(gyro_prop_q).T
        self.F[:3, :3] = RT.reshape((3,3))

        self.P = self.F.T @ self.P @ self.F + self.Q

        if self.P[2][2] > 1e-6:
            self.P[2][2] = 1e-6

    def update(self, accel:np.array):
        linear_accel_var = np.square(np.linalg.norm(accel) - 1)
        V = self.accel_var + linear_accel_var * 10 * np.eye(3,3)

        RT_q_g = quaternion.rotate_vectors(self.state.q.conjugate(), np.array([0,0,-1], dtype=np.float32))

        #H = skew(RT_q_g)

        w,x,y,z = self.state.q.w, self.state.q.x, self.state.q.y, self.state.q.z

        H_x = np.array([ 
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
        self.H = H_x @ Q_dtheta

        K = self.P @ self.H.T @ np.linalg.inv((self.H @ self.P @ self.H.T + V))
        self.error_state.statevect += K @ (accel - (RT_q_g.reshape(3,1)))
        I = np.eye(3,3, dtype=np.float32)
        self.P = (I - K @ self.H) @ self.P @ (I - K @ self.H).T + K @ V @ K.T

        self.debug_froben_norm = np.max(self.P)
        self.max_index = np.unravel_index(np.argmax(self.P), self.P.shape)

    def inject(self):
        self.state.update(self.error_state)
        self.error_state.reset()

if __name__ == "__main__":
    q = quaternion.from_rotation_vector(np.array([1e-5,1e-5,1e-5]))
    w, x, y, z = q.w, q.x, q.y, q.z
    

    print(q)
    H_x = np.array([ 
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
    H = H_x @ Q_dtheta
    print(H)
