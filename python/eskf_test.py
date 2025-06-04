
from eskf_s import *
import time
import numpy as np
import quaternion

def quat_to_euler_zyx(q):
    w, x, y, z = q.w, q.x, q.y, q.z

    # Yaw (Z)
    psi = np.arctan2(2*(w*z + x*y), 1 - 2*(y**2 + z**2))

    # Pitch (Y)
    sin_theta = 2*(w*y - z*x)
    theta = np.arcsin(np.clip(sin_theta, -1.0, 1.0))  # safe arcsin

    # Roll (X)
    phi = np.arctan2(2*(w*x + y*z), 1 - 2*(x**2 + y**2))

    return np.array([psi, theta, phi])

reader = IMUReader()


last_read = time.perf_counter()
eskf = Eskf()

covar_sum = 0
covar_samples = 0
covar = 0
while True: 
    data = reader.read()
    elapsed = time.perf_counter() - last_read
    npdata = np.array(data) 
    #npdata = np.array([0.0,0.0,0.0, 0.0,-0.7071,-0.7071])
    eskf.propagate(npdata[:3].reshape(3,1), DT)
    eskf.update(npdata[3:].reshape((3,1)))
    if elapsed > 50*1e-3:
        print("\033[2J\033[H", end='')
        last_read = time.perf_counter()
        euler = quat_to_euler_zyx(eskf.state.q) * 180 / 3.14159
        print(f"{euler} \n \n")
        #print(f"\r{eskf.P} \n \n",end='', flush=True)
        #print(f"\r{eskf.H} \n \n",end='', flush=True)
        #print(f"\r{eskf.state.q} \n \n",end='', flush=True)
        #print(f"\r{euler} \n \n",end='', flush=True)
        #print(f"{eskf.state.wb} \n \n")
        #print(f"{eskf.state.ab} \n \n")
        #print(f"{eskf.max_index}   {eskf.debug_froben_norm}")
    
    eskf.inject()