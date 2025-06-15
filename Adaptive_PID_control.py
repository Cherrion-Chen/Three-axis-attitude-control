from ctypes import *
import numpy as np

dll = cdll.LoadLibrary("./quaternion.dll")

# 设置函数的参数类型
dll.set_I.argtypes = (c_double, c_double, c_double)
dll.f.argtypes = (POINTER(c_double), POINTER(c_double))
dll.multiply.argtypes = (POINTER(c_double), POINTER(c_double))

# 设置`add`函数的返回类型
dll.f.restype = POINTER(c_double)
dll.multiply.restype = POINTER(c_double)

I = np.array([1., 1, 1])
dll.set_I(*I)

def quat_distance(q1, q2):
    dot = np.clip(np.abs(np.dot(q1, q2)), 0., 1.)
    return np.acos(dot)

def norm_q(q, epsilon=1e-8):
    return q / np.sqrt(np.sum(q*q)+epsilon)

def q_ddot(q, q_dot, M):
    x = (c_double * 8)(*(list(q)+list(q_dot)))
    u = (c_double * 3)(*M)
    op = dll.f(x, u)
    return np.array(op[:4])

def quaternion_multiply(q1, q2):
    '''
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
    y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
    z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
    '''
    q1 = (c_double * 4)(*q1)
    q2 = (c_double * 4)(*q2)
    op = dll.multiply(q1, q2)
    return np.array(op[:4])

def quaternion_conjugate(q):
    return np.array([1, -1, -1, -1]) * q

def quaternion_to_omega(q, q_dot):
    q_inv = quaternion_conjugate(q)
    omega_q = 2 * quaternion_multiply(q_inv, q_dot)
    return omega_q[1:]

def quaternion_error(q, q_d):
    q_conj = quaternion_conjugate(q)
    q_e = quaternion_multiply(q_d, q_conj)
    if q_e[0] < 0:
        q_e = -q_e
    return q_e

def quaternion_slerp(qa, qb, t):
    """Quaternion spherical linear interpolation"""
    qm = np.zeros(4)
    cos_half_theta = np.dot(qa, qb)
    
    if cos_half_theta < 0:
        qb = -qb
        cos_half_theta = -cos_half_theta
    
    half_theta = np.arccos(cos_half_theta)
    sin_half_theta = np.sqrt(1.0 - cos_half_theta**2)
    
    if np.abs(sin_half_theta) < 0.001:
        return (1-t)*qa + t*qb
    
    ratio_a = np.sin((1-t)*half_theta)/sin_half_theta
    ratio_b = np.sin(t*half_theta)/sin_half_theta
    return ratio_a*qa + ratio_b*qb

def velocity_required(q, q_dot, qd, qd_dot, Kp_outer=2, Kd_outer=0.5, alpha=0.2):
    """
    Outer loop of the cascade control:
        ————generating the transition attitude and quarternion velocity.
    Input values：
        q: Current attitude (4,)
        q_dot: Current quarternion velocity (4,)
        qd: Target attitude (4,)
        qd_dot: Target quarternion velocity (4,)
        Kp_outer: Propertion gain of the outer loop
        Kd_outer: Differential gain of the outer loop
        alpha: Smooth factor
    Return values：
        qd_required: Transition target attitude (4,)
        qd_dot_required: Transition target quarternion velocity (4,)
    """
    q, qd = norm_q(q), norm_q(qd)
    q_e = quaternion_multiply(qd, quaternion_conjugate(q))
    if q_e[0] < 0:
        q_e = -q_e
    
    qd_required = quaternion_slerp(q, qd, alpha)
    omega_desired = Kp_outer * q_e[1:] - Kd_outer * quaternion_to_omega(q, q_dot)
    qd_dot_required = 0.5 * quaternion_multiply(qd_required, np.insert(omega_desired, 0, 0))
    qd_dot_required = alpha * qd_dot + (1 - alpha) * qd_dot_required
    
    return qd_required, qd_dot_required

class velocity_tracking():
    """
    Controller for tracking quarternion velocity
    Input values：
        q: Current attitude (4,)
        q_dot: Current quarternion velocity (4,)
        qd: Target attitude (4,)
        qd_dot: Target quarternion velocity (4,)
        K: Gain matrix (3, 3)
    Return values：
        tau: Control torque (3,)
    """
    def __init__(self, Kp=3., Ki=0.5, s=1e-2):
        self.Kp = Kp
        self.Ki = Ki
        self.s = s
        self.error_integral = 0
    def __call__(self, q, q_dot, qd, qd_dot):
        # Normalization
        q = q / np.linalg.norm(q)
        qd = qd / np.linalg.norm(qd)
    
        # Calculation of the current and the target angular velocities
        omega = 2 * quaternion_multiply(quaternion_conjugate(q), q_dot)[1:]
        omega_d = 2 * quaternion_multiply(quaternion_conjugate(qd), qd_dot)[1:]
    
        # Error of angular velocity
        omega_error = omega_d - omega
        self.error_integral += self.s * omega_error
        # Adjusting the gain dynamically
        error_norm = np.linalg.norm(omega_error)
        adaptive_gain = 1 + 0.5 * np.tanh(error_norm - 1)  # Nonliear adjustment
    
        tau = adaptive_gain * self.Kp * omega_error + self.Ki * self.error_integral
    
        return tau

if __name__ == "__main__":
    t, s = 0, 0.01
    q = norm_q(np.array([1., -1, -1, 2]))
    v = np.array([0., .5, 0, 0])
    qd = np.array([1., 0, 0, 0])
    qd_dot = np.array([0.]*4)
    controller = velocity_tracking(Kp=8, Ki=0.5, s=s)
    M = 0
    wp = quaternion_to_omega(q, v)
    w = wp

    tt, q_list, angular_error, L2_error = [], [], [], []
    while t <= 30:
        required = velocity_required(q, v, qd, qd_dot)
        M = controller(q, v, *required)
        wp = w
        w = quaternion_to_omega(q, v)
        a = q_ddot(q, v, M)
        v += s*(a+np.array([0.2]*4))
        q = norm_q(q + s*v)
        t += s
        tt.append(t)
        q_list.append(q)
        angular_error.append(quat_distance(qd, q))
        L2_error.append(np.sum((qd-q)**2))

    import matplotlib.pyplot as plt
    from visualizer import anim
    
    plt.title("Angular error")  
    plt.plot(tt, angular_error)
    plt.show()

    plt.title("Quaternion L2 error")
    plt.plot(tt, L2_error)
    plt.show()

    anim(q_list, s)
