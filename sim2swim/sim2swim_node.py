#!/usr/bin/env python3

import numpy as np
import rclpy
import os
import torch
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

os.environ["KMP_DUPLICATE_LIB_OK"] = "TRUE"

from geometry_msgs.msg import WrenchStamped, QuaternionStamped, Vector3Stamped
from std_msgs.msg import Bool

class Controller(Node):
    def __init__(self):
        
        super().__init__('Sim2Swim')
        
        self.declare_topic_parameters()
        self.setup()

        self.publishers()
        self.subscribers()
        
        
    def declare_topic_parameters(self):
        defaults = {
            'desired_velocity_body': '/guidance/desired_velocity_body',
            'desired_quaternion':    '/guidance/quaternion',
            'angular_velocity':      '/bluerov2/angular_velocity',
            'quaternion':            '/bluerov2/quaternion',
            'arm_status':            '/bluerov2/arm_status',
            'in_auto':               '/bluerov2/in_auto',
            'velocity_body':         '/observer/estimate/linear_velocity_body',
            'tau_pwm':               '/controller/tau_pwm',
        }
        self.topics = {
            key: self.declare_parameter(f'topics.{key}', default).value
            for key, default in defaults.items()
        }

    def setup(self):
        pkg_path = get_package_share_directory('sim2swim')
        policy_path = os.path.join(pkg_path,'sim2swim','policy.pt')
        self.policy = torch.jit.load(policy_path, map_location="cpu")
        self.policy.eval()

       
        self.time_last_message = self.get_clock().now().nanoseconds / 1e9
        self.start_time = self.get_clock().now().nanoseconds / 1e9
        
        self.quat = np.array([1,0,0,0])
        self.quat_d = np.array([1,0,0,0])
        self.pos_n = np.zeros(3)
        self.pos_d_n = np.zeros(3)
        self.vel_d = np.zeros(3)
        self.vel_b = np.zeros(3)
        self.ang_vel_b = np.zeros(3)

        self.integral_vel_error = np.zeros(3)
        self.integral_attitude_error = torch.zeros(3)
        self.auto = False
        self.armed = False

        self.dvl_lever_arm = np.zeros(3)
        self.dvl_rotation_matrix = np.identity(3)

        self.SAT = 3


        self.get_logger().info("[Sim2Swim] Setup complete")
    def subscribers(self):

        t = self.topics
        self.sub_guidance        = self.create_subscription(Vector3Stamped,    t['desired_velocity_body'], self.callback_guidance,       10)
        self.sub_ang_vel         = self.create_subscription(Vector3Stamped,    t['angular_velocity'],      self.callback_ang_vel,        10)
        self.sub_quaternion      = self.create_subscription(QuaternionStamped, t['quaternion'],            self.callback_quaternion,     10)
        self.sub_quaternion_d    = self.create_subscription(QuaternionStamped, t['desired_quaternion'],    self.callback_quaternion_d,   10)
        self.sub_armed           = self.create_subscription(Bool,              t['arm_status'],            self.callback_armed,          10)
        self.sub_auto            = self.create_subscription(Bool,              t['in_auto'],               self.callback_auto,           10)
        self.sub_velocity_body   = self.create_subscription(Vector3Stamped,    t['velocity_body'],         self.callback_velocity_body,  10)


        
    def publishers(self):      
        self.publisher_controller_state_pwm  = self.create_publisher(WrenchStamped,     self.topics['tau_pwm'],  1)
        
    def callback_auto(self,data):
        if (not self.auto) and data.data:
            self.reset_integral()
            self.start_time = self.get_clock().now().nanoseconds / 1e9 #Update start time to current time if not in auto
        self.auto = data.data

    def callback_armed(self,data):
        if (not self.armed) and data.data:
            self.reset_integral()
            self.start_time = self.get_clock().now().nanoseconds / 1e9 #Update start time to current time if not in auto
        self.armed = data.data

    def callback_ang_vel(self, data):
        p = data.vector.x
        q = data.vector.y
        r = data.vector.z
        self.ang_vel_b = np.array([p,q,r])


    def callback_velocity_body(self, data):

        self.vel_b = np.array([data.vector.x, data.vector.y, data.vector.z], dtype=np.float64)
        self.apply_policy()

        self.time_last_message = data.header.stamp.sec + data.header.stamp.nanosec * 1e-9

    def callback_quaternion(self, data):

        w = data.quaternion.w
        x = data.quaternion.x
        y = data.quaternion.y
        z = data.quaternion.z

        self.quat = np.array([w,x,y,z])

        self.apply_policy()

        self.time_last_message = data.header.stamp.sec + data.header.stamp.nanosec * 1e-9

    def callback_quaternion_d(self, data):
        
        w = data.quaternion.w
        x = data.quaternion.x
        y = data.quaternion.y
        z = data.quaternion.z

        self.quat_d = np.array([w,x,y,z])
        
        self.apply_policy()

        self.time_last_message = data.header.stamp.sec + data.header.stamp.nanosec * 1e-9


    def reset_integral(self):
        self.integral_vel_error = np.zeros(3)
        self.integral_attitude_error = torch.zeros(3)

    def callback_guidance(self, data):

        u_d = data.vector.x
        v_d = data.vector.y
        w_d = data.vector.z

        self.vel_d = np.array([u_d,v_d,w_d])

    def apply_policy(self):
        if not self.auto or not self.armed:
            return

        quat_error = torch.tensor(quatunique(quatprod(quatconj(self.quat_d),self.quat)), dtype=torch.float32)
        quat_error = quat_error / torch.norm(quat_error)
       
        dt = self.get_clock().now().nanoseconds / 1e9 - self.time_last_message

        self.integral_vel_error += self.auto*(self.vel_d - self.vel_b)*dt
        self.integral_vel_error = np.clip(self.integral_vel_error,-self.SAT,self.SAT)
        integral_vel_error = torch.tensor(self.integral_vel_error, dtype=torch.float32)

        vel_error = torch.tensor(self.vel_d - self.vel_b, dtype=torch.float32)
        ang_vel = torch.tensor(self.ang_vel_b,dtype=torch.float32)

        self.integral_attitude_error += self.auto*quat_error[1:4]*dt
        obs = torch.cat([quat_error, vel_error, ang_vel, integral_vel_error,self.integral_attitude_error], dim=0)

        with torch.no_grad():
           action = self.policy(obs) 
        action = np.clip(action.numpy(),-1,1)      

        msg_pwm = WrenchStamped()
        msg_pwm.header.stamp = self.get_clock().now().to_msg()

        msg_pwm.wrench.force.x  = unit2pwm(action[0])
        msg_pwm.wrench.force.y  = unit2pwm(action[1])
        msg_pwm.wrench.force.z  = unit2pwm(action[2])
        msg_pwm.wrench.torque.x = unit2pwm(action[3])
        msg_pwm.wrench.torque.y = unit2pwm(action[4])
        msg_pwm.wrench.torque.z = unit2pwm(action[5])

        self.publisher_controller_state_pwm.publish(msg_pwm)
   

def unit2pwm(unit):
    return unit * 400  + 1500
        

  
def euler2quat(phi,theta,psi):
    cy = np.cos(psi * 0.5)
    sy = np.sin(psi * 0.5)
    cp = np.cos(theta * 0.5)
    sp = np.sin(theta * 0.5)
    cr = np.cos(phi * 0.5)
    sr = np.sin(phi * 0.5)

    q = np.array([cy * cp * cr + sy * sp * sr,
         cy * cp * sr - sy * sp * cr,
         sy * cp * sr + cy * sp * cr,
         sy * cp * cr - cy * sp * sr])

    q = q/(np.sqrt(np.dot(q,q)))
    return q

def quatprod(q1, q2):
    eta1 = q1[0] 
    eps1 = q1[1:4]
    eta2 = q2[0]
    eps2 = q2[1:4]
    q = np.hstack(( eta1 * eta2 - np.dot(eps1, eps2),eta2 * eps1 + eta1 * eps2 + np.cross(eps1, eps2) ))
    return q

def quatinv(q):
    qinv = np.copy(q)
    qinv[1:4] = -qinv[1:4]
    qinv = qinv/(qinv@qinv)
    return qinv

def quatconj(q):
    conj = np.copy(q)
    conj[1:4] = -conj[1:4]
    return conj

def Smtrx(a):
    """
    Skew-symmetric matrix for a 3D vector a such that
    S(a) =  - S(a)' and the cross product can be expressed as
    a x b = S(a) * b. The inverse can be computed as: vex(S(a)) = a.
    """
    return np.array([[0, -a[2], a[1]],
                      [a[2], 0, -a[0]],
                      [-a[1], a[0], 0]], dtype=np.float32)

def Rquat(q):
    """
    Computes the rotation matrix R in SO(3) from a quaternion q=[w, x, y, z].
    The quaternion must be normalized, i.e., ||q|| = 1.
    """
    tol = 1e-6
    if np.linalg.norm(q) < tol:
        raise ValueError("Quaternion norm is too small.")
    
    q = q / np.linalg.norm(q)  # Normalize the quaternion
    
    eta = q[0]
    eps = q[1:4]

    S = Smtrx(eps)
    R = np.eye(3) + 2 * eta * S + 2 * S @ S

    return R

def quatrotate(q,v):
    #rotate a vector v by the quaternion q
    eta = q[0]
    eps = q[1:4]
    t = 2* np.cross(eps,v)
    v_m = v + eta*t + np.cross(eps,t)    

    return v_m

def quatunique(q):
    #Returns the quaternion representation with positive real part
    q1 = np.copy(q)
    if q1[0] < 0:
        q1 = -q1
    return q1

def wrap2pi(a):
    a = (a + np.pi)%(2*np.pi) - np.pi
    return a


def main():
    rclpy.init()
    node = Controller()

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()   


    
    