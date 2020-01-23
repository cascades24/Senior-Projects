from __future__ import division
"""
Attempts to track {x,y,theta}

Publishes estimated odom on /ekf_estimate
"""
import rospy
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
import numpy as np
from std_msgs.msg import Float32MultiArray
import threading
from scipy import integrate

THETA_INDEX = 2
SPEED_INDEX = 3
THETA_DOT_INDEX = 4

class EKF():
    def __init__(self):
        self.lock = threading.Lock()
        self.last_control_time = rospy.get_time()
        self.control_queue = [0,0]
        self.meas_queue = []
        self.mean = np.array([[]])
        self.sigma = np.array([[1, 0, 0],[0,1,0],[0,0,0.1]])
        self.last_update_time = None

        rospy.Subscriber("/turtle1/meas", Float32MultiArray, self.meas_callback)
        print("...waiting for first measurement")
        rospy.wait_for_message("/turtle1/meas", Float32MultiArray)
        rospy.Subscriber("/turtle1/cmd_vel", Twist, self.control_callback)
        self.mean_pub = rospy.Publisher("/turtle1/ekf_mean", Float32MultiArray, queue_size=10)
        self.cov_pub = rospy.Publisher("/turtle1/ekf_cov", Float32MultiArray, queue_size=10)

        self.motion_noise = np.eye(3) * np.array([[0.05,0.05,0.01]]).T
        self.meas_noise = np.array([[1,0,0],[0,1,0],[0,0,0.25]])

    def control_callback(self, msg):
        self.lock.acquire(True)
        self.last_control_time = rospy.get_time()
        s = msg.linear.x
        theta_dot = msg.angular.z
        self.control_queue = [s, theta_dot]
        self.lock.release()
    
    def normalize_angle(self, angle):
        """ Normalize the angle from -pi to pi """
        while angle <= np.pi:
            angle += 2*np.pi
        while angle >= np.pi:
            angle -= 2*np.pi
        return angle

    def run_filter(self):
        self.lock.acquire(True)

        """ Prediction Step """
        if self.mean.size == 0:
            self.mean = np.array([self.meas_queue]).T
            print(self.mean)
            self.last_update_time = rospy.get_time()
            self.lock.release()
            return

        # Turtle stops moving 1s after last control input,
        if rospy.get_time() - self.last_control_time >= 1.0:
            # print("Emptying control queue")
            self.control_queue = [0,0] # empty the queue
        
        # Calculate delta_t & set new update time
        now = rospy.get_time()
        dt = now - self.last_update_time
        self.last_update_time = now

        s = self.control_queue[0] # fwd velocity / speed
        theta_dot = self.control_queue[1] # angular velocity
        x_initial = float(self.mean[0])
        y_initial = float(self.mean[1])
        theta_initial = float(self.mean[2])

        # Runge-Kutta integrate mean prediction
        def dynamics(t, z):
            _x_dot = s * np.cos(z[2])
            _y_dot = s * np.sin(z[2])
            _theta_dot = theta_dot
            return np.array([_x_dot, _y_dot, _theta_dot])

        t_init, t_final = 0, dt
        z_init = np.array([x_initial, y_initial, theta_initial])
        r = integrate.RK45(dynamics, t_init, z_init, t_final)
        while r.status == "running":
            status = r.step()
        mean_bar = np.reshape(r.y, (r.y.size, 1))
        mean_bar[2] = self.normalize_angle(mean_bar[2])
        
        # Euler Intergrate the covariance
        # Jacobian of motion model, use Euler Integration (Runge-kutta for mean estimate)
        G = np.array([[1, 0, -dt*s*np.sin(theta_initial)],\
                      [0, 1, dt*s*np.cos(theta_initial)],\
                      [0, 0, 1]])
        sigma_bar = np.dot( np.dot(G, self.sigma), G.T) + (self.motion_noise * dt)

        # this filter updates at a certain rate, the simpler approach is to run the filter on every measurement received
        if not self.meas_queue: 
            self.mean = mean_bar
            self.sigma = sigma_bar
        else:
            x_meas = self.meas_queue[0]
            y_meas = self.meas_queue[1]
            theta_meas = self.meas_queue[2]
            meas = np.array([[x_meas,y_meas,theta_meas]]).T

            H = np.eye(3)
            tmp = np.dot(np.dot(H, sigma_bar), H.T) + self.meas_noise
            tmp_inv = np.linalg.inv(tmp)
            K = np.dot( np.dot( sigma_bar, H), tmp_inv )
            inn = meas - mean_bar
            self.mean = mean_bar + np.dot(K, inn)
            self.sigma = np.dot( np.eye(3) - np.dot(K,H), sigma_bar)
            self.meas_queue = [] # empty the queue
        print(self.mean)
        print(self.sigma)
        print("--------")
        self.pub_estimates(self.mean, self.sigma)
        
        self.lock.release()

    def meas_callback(self, msg):
        self.lock.acquire(True)
        self.meas_queue = msg.data
        self.lock.release()

    def pub_estimates(self, mean, cov):
        mean_msg = Float32MultiArray()
        mean_msg.data = [float(i) for i in mean]
        self.mean_pub.publish(mean_msg)
        cov_msg = Float32MultiArray()
        cov_msg.data = [float(i) for i in cov.flatten()]
        self.cov_pub.publish(cov_msg)


    def debug_prediction(self, G, mu_old, sigma_old, mu_bar, sigma_bar):
        print("------------------------------------------------")
        print("Debug Prediction Step:")
        print("G: " + str(G.shape))
        print(G)
        print("mu_old: " + str(mu_old.shape))
        print(mu_old)
        print("sigma_old: " + str(sigma_old.shape))
        print(sigma_old)
        print("mu_bar: " + str(mu_bar.shape))
        print(mu_bar)
        print("sigma_bar: " + str(sigma_bar.shape))
        print(sigma_bar)

    def debug_correction(self, H, h, z, mu_bar, sigma_bar, tmp, tmp_inv, sigma_bar_HT_prod, K, innovation, tmp2, mu, sigma):
        print("------------------------------------------------")
        print("Debug Correction Step:")
        print("H: " + str(H.shape))
        print(H)
        print("h: " + str(h.shape))
        print(h)
        print("z: " + str(z.shape))
        print(z)
        print("mu_bar: " + str(mu_bar.shape))
        print(mu_bar)
        print("sigma_bar: " + str(sigma_bar.shape))
        print(sigma_bar)

        print("tmp: " + str(tmp.shape))
        print(tmp)

        print("tmp_inv: " + str(tmp_inv.shape))
        print(tmp_inv)

        print("sigma_bar_HT_prod: " + str(sigma_bar_HT_prod.shape))
        print(sigma_bar_HT_prod)

        print("K: " + str(K.shape))
        print(K)

        print("innovation: " + str(innovation.shape))
        print(innovation)

        print("tmp2: " + str(tmp2.shape))
        print(tmp2)

        print("mu: " + str(mu.shape))
        print(mu)

        print("sigma: " + str(sigma.shape))
        print(sigma)


if __name__ == "__main__":
    rospy.init_node("ekf")
    ekf = EKF()
    r = rospy.Rate(25)
    while not rospy.is_shutdown():
        ekf.run_filter()
        r.sleep()
