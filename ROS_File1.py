#!/usr/bin/env python3
from math import pi, sqrt, atan2, cos, sin
from turtle import position
import numpy as np
from numpy import NaN
import rospy
import tf
from std_msgs.msg import Empty, Float32
from nav_msgs.msg import Odometry
from mav_msgs.msg import Actuators
from geometry_msgs.msg import Twist, Pose2D
import pickle
import os

from sympy import symbols, sin, cos, tan, Matrix
import sympy
import math
import numpy as np
import QuinticTraj as QT


class Quadrotor():
    def __init__(self):
        # publisher for rotor speeds
        self.motor_speed_pub = rospy.Publisher("/crazyflie2/command/motor_speed", Actuators, queue_size=10)
        # subscribe to Odometry topic
        self.odom_sub = rospy.Subscriber("/crazyflie2/ground_truth/odometry",Odometry, self.odom_callback)
        self.t0 = None
        self.t = None
        self.t_series = []
        self.x_series = []
        self.y_series = []
        self.z_series = []
        self.mutex_lock_on = False
        rospy.on_shutdown(self.save_data)
        # TODO: include initialization codes if needed

        ten6 = 10**-6
        self.m, self.l, self.Ix, self.Iy, self.Iz, self.Ip  = 0.027, 0.046, 16.5717*ten6, 16.5717*ten6, 29.2616*ten6, 12.65625*ten6
        self.kF, self.kM, self.w_max, self.w_min, self.g = 1.26192*10**-8, 5.9645*10**-3, 2618, 0, 9.81

        # setting initial inputs to 0 
        self.u1, self.u2, self.u3, self.u4 = 0, 0, 0, 0

    def traj_evaluate(self):
        # TODO: evaluating the corresponding trajectories designed in Part 1 
        # to return the desired positions, velocities and accelerations
        v0 = [0,0,0]
        vf = [0,0,0]
        acc0 = [0,0,0]
        accf = [0,0,0]
        points = [[0,0,0], [0, 0, 1], [1, 0, 1], [1, 1, 1], [0, 1, 1], [0, 0, 1]]
        traj = []
        tspan = [[0, 5], [5, 20], [20, 35], [35, 50], [50, 65]]
        for i in range(len(points)-1):
            traj_obj = QT.QuinticTraj(tspan[i], points[i], points[i+1], v0, vf, acc0, accf)
            traj.append(traj_obj.GenerateQuinticTraj())

        self.x_d = traj_obj.calculate_position(self.t, traj[0][0])
        self.y_d = traj_obj.calculate_position(self.t, traj[0][1])
        self.z_d = traj_obj.calculate_position(self.t, traj[0][2])

        self.dx_d = traj_obj.calculate_velocity(self.t, traj[0][0])
        self.dy_d = traj_obj.calculate_velocity(self.t, traj[0][1])
        self.dz_d = traj_obj.calculate_velocity(self.t, traj[0][2])

        self.ddx_d = traj_obj.calculate_acceleration(self.t, traj[0][0])
        self.ddy_d = traj_obj.calculate_acceleration(self.t, traj[0][1])
        self.ddz_d = traj_obj.calculate_acceleration(self.t, traj[0][2])

        
        
    def smc_control(self, xyz, xyz_dot, rpy, rpy_dot):
        # obtain the desired values by evaluating the corresponding
        # trajectories
        self.traj_evaluate()
        # We have x_d, y_d, z_d we calculate the theta_d and phi_d from the relations given between x,y,z
        Kp, Kd = 0.1, 0.1 # PD control Kp, Kd
        # x,y = 1, 1
        # dx, dy = 0, 0
        Fx = self.m * (-Kp * (xyz[0,0] - self.x_d) - Kd * (xyz_dot[0,0]-self.dx_d) + self.ddx_d) 
        Fy = self.m * (-Kp * (xyz[1,0] - self.y_d) - Kd * (xyz_dot[1,0]-self.dy_d) + self.ddy_d)
        #print(Fx) 
        self.theta_d = sympy.asin(*Fx/self.u1) # U1 design karke idhar kaise dale dekho broo
        self.phi_d = sympy.asin(*-Fy/self.u1)
        self.psy_d, self.dphi_d, self.dtheta_d, self.dpsy_d, self.ddphi_d, self.ddtheta_d, self.ddpsy_d = 0,0,0,0,0,0,0 
        t1,t2,t3 = 1/(4*self.kF), math.sqrt(2)/(4*self.kF*self.l), 1/(4*self.kM*self.kF)

        # TODO: implement the Sliding Mode Control laws designed in Part 2 to
        # calculate the control inputs "u"

        # NOTE :: motor_vel is w1 w2 w3 w4 array so check it and change variable names for consistency
        w1 = sympy.sqrt( Matrix([t1,-t2,-t2,-t3]).dot( Matrix([self.u1,self.u2,self.u3,self.u4])) )
        w2 = sympy.sqrt( Matrix([t1,-t2, t2, t3]).dot( Matrix([self.u1,self.u2,self.u3,self.u4])) )
        w3 = sympy.sqrt( Matrix([t1, t2, t2,-t3]).dot( Matrix([self.u1,self.u2,self.u3,self.u4])) )
        w4 = sympy.sqrt( Matrix([t1, t2,-t2, t3]).dot( Matrix([self.u1,self.u2,self.u3,self.u4])) )
        Omega = w1-w2+w3-w4

        # REMARK: wrap the roll-pitch-yaw angle errors to [-pi to pi]
        # TODO: convert the desired control inputs "u" to desired rotor
        laambda = 1
        q_d = np.array([self.x_d, self.y_d, self.z_d, self.phi_d, self.theta_d, self.psy_d]).T
        dq_d = np.array([self.dx_d, self.dy_d, self.dz_d, self.dphi_d, self.dtheta_d, self.dpsy_d]).T
        ddq_d = np.array([self.ddx_d, self.ddy_d, self.ddz_d, self.ddphi_d, self.ddtheta_d, self.ddpsy_d]).T
        e = xyz - q_d
        de = xyz_dot - dq_d
        phi, theta, psy = rpy
        dphi, dtheta, dpsy = rpy_dot
        ddx = (self.u1/self.m) * ( cos(phi)*sin(theta)*cos(psy) + sin(phi)*sin(psy) )
        ddy = (self.u1/self.m) * ( cos(phi)*sin(theta)*cos(psy) - sin(phi)*sin(psy) )
        ddz = (self.u1/self.m) * ( cos(phi)*cos(theta) ) -self.g
        ddphi = dtheta*dpsy + (self.Iy-self.Iz)/self.Ix - self.Ip/self.Ix*Omega*dtheta + 1/self.Ix * self.u2
        ddtheta = dphi*dpsy + (self.Iz-self.Ix)/self.Iy - self.Ip/self.Iy*Omega*dphi + 1/self.Iy * self.u3
        ddpsy = dtheta*dphi + (self.Ix-self.Iy)/self.Iz - 1/self.Iz * self.u4

        f4 = dtheta*dpsy((self.Iy - self.Iz)/self.Ix) - (self.Ip/self.Ix)*Omega*dtheta
        f5 = dphi*dpsy((self.Iz - self.Ix)/self.Iy) - (self.Ip/self.Iy)*Omega*dphi
        f6 = dphi*dtheta((self.Ix-self.Iy)/self.Iz)
        f = np.array([0, 0, -self.g, f4, f5, f6]).T
        k = np.ones((6,1))
        
        s = de + laambda * e

        u_r = -k * np.sign(s)
        u = (-f + ddq_d) - laambda * de + u_r
        self.u1 = u[2]
        self.u2 = u[3]
        self.u3 = u[4]
        self.u4 = u[5]
        # velocities "motor_vel" by using the "allocation matrix"
        # TODO: maintain the rotor velocities within the valid range of [0 to
        # 2618]
        motor_vel = [[w1], [w2], [w3], [w4]]
        # publish the motor velocities to the associated ROS topic
        motor_speed = Actuators()
        motor_speed.angular_velocities = [motor_vel[0,0], motor_vel[1,0], motor_vel[2,0], motor_vel[3,0]]
        self.motor_speed_pub.publish(motor_speed)

    # odometry callback function (DO NOT MODIFY)
    def odom_callback(self, msg):
        if self.t0 == None:
            self.t0 = msg.header.stamp.to_sec()
        self.t = msg.header.stamp.to_sec() - self.t0

        # convert odometry data to xyz, xyz_dot, rpy, and rpy_dot
        w_b = np.asarray([[msg.twist.twist.angular.x], [msg.twist.twist.angular.y], [msg.twist.twist.angular.z]])
        v_b = np.asarray([[msg.twist.twist.linear.x], [msg.twist.twist.linear.y], [msg.twist.twist.linear.z]])
        xyz = np.asarray([[msg.pose.pose.position.x], [msg.pose.pose.position.y], [msg.pose.pose.position.z]])
        q = msg.pose.pose.orientation
        T = tf.transformations.quaternion_matrix([q.x, q.y, q.z, q.w])
        T[0:3, 3] = xyz[0:3, 0]
        R = T[0:3, 0:3]
        xyz_dot = np.dot(R, v_b)
        rpy = tf.transformations.euler_from_matrix(R, 'sxyz')
        rpy_dot = np.dot(np.asarray([
        [1, np.sin(rpy[0])*np.tan(rpy[1]), np.cos(rpy[0])*np.tan(rpy[1])],
        [0, np.cos(rpy[0]), -np.sin(rpy[0])],
        [0, np.sin(rpy[0])/np.cos(rpy[1]), np.cos(rpy[0])/np.cos(rpy[1])]
        ]), w_b)
        rpy = np.expand_dims(rpy, axis=1)
        # store the actual trajectory to be visualized later
        if (self.mutex_lock_on is not True):
            self.t_series.append(self.t)
            self.x_series.append(xyz[0, 0])
            self.y_series.append(xyz[1, 0])
            self.z_series.append(xyz[2, 0])
        
        # call the controller with the current states
        self.smc_control(xyz, xyz_dot, rpy, rpy_dot)

    # save the actual trajectory data
    def save_data(self):
        # TODO: update the path below with the correct path
        with open("/home/sfarzan/rbe502_project/src/project/scripts/log.pkl","wb") as fp:
            self.mutex_lock_on = True
            pickle.dump([self.t_series,self.x_series,self.y_series,self.z_series], fp)


if __name__ == '__main__':
    rospy.init_node("quadrotor_control")
    rospy.loginfo("Press Ctrl + C to terminate")
    whatever = Quadrotor()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down")