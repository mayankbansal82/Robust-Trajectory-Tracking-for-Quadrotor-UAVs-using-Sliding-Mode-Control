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
class Quadrotor():
    def __init__(self):
    # publisher for rotor speeds
        self.motor_speed_pub = rospy.Publisher("/crazyflie2/command/motor_speed", Actuators, queue_size=100)

        # subscribe to Odometry topic
        self.odom_sub = rospy.Subscriber("/crazyflie2/ground_truth/odometry", Odometry, self.odom_callback)

        self.omega = 0
        self.t0 = None
        self.t = None
        self.t_series = []
        self.x_series = []
        self.y_series = []
        self.z_series = []
        self.mutex_lock_on = False
        rospy.on_shutdown(self.save_data)

        # TODO: include initialization codes if needed
        self.z_d = 0
        self.z_d_dot = 0
        self.z_d_ddot = 0

        self.x_d = 0
        self.x_d_dot = 0
        self.x_d_ddot = 0

        self.y_d = 0
        self.y_d_dot = 0
        self.y_d_ddot = 0

        self.phi_d_dot = 0
        self.theta_d_dot = 0
        self.psi_d_dot = 0

        self.phi_d_ddot = 0
        self.theta_d_ddot = 0
        self.psi_d_ddot = 0

        self.psi_d = 0


    def sat(self,val,phi): 
        sn = 0
        if val < 0:
            sn = -1
        elif val > 0:
            sn = 1
        if abs(val)>phi:
            return sn
        return val/phi

        

    def traj_evaluate(self):
        # TODO: evaluating the corresponding trajectories designed in Part 1 to return the desired positions, velocities and accelerations
        t = self.t
 
        if t < 5:
            self.z_d = 6*t**5/3125 - 3*t**4/125 + 2*t**3/25
            self.z_d_dot = 6*t**4/625 - 12*t**3/125 + 6*t**2/25
            self.z_d_ddot = 24*t**3/625 - 36*t**2/125 + 12*t/25

            self.x_d = 0
            self.x_d_dot = 0
            self.x_d_ddot = 0

            self.y_d = 0
            self.y_d_dot = 0
            self.y_d_ddot = 0

        elif t < 20:
            self.z_d = 1
            self.z_d_dot = 0
            self.z_d_ddot = 0
            t2 = t*t
            t3 = t2*t
            t4 = t3*t
            t5 = t4*t

            self.x_d = (2*t5)/253125 - t4/2025 + (22*t3)/2025 - (8*t2)/81 + (32*t)/81 - 47/81
            self.x_d_dot = (2*t4)/50625 - (4*t3)/2025 + (22*t2)/675 - (16*t)/81 + 32/81
            self.x_d_ddot = (8*t3)/50625 - (4*t2)/675 + (44*t)/675 - 16/81

            self.y_d = 0
            self.y_d_dot = 0
            self.y_d_ddot = 0

        elif t < 35:
            self.z_d = 1
            self.z_d_dot = 0
            self.z_d_ddot = 0
            t2 = t*t
            t3 = t2*t
            t4 = t3*t
            t5 = t4*t

            self.x_d = 1
            self.x_d_dot = 0
            self.x_d_ddot = 0

            self.y_d = (2*t5)/253125 - (11*t4)/10125 + (118*t3)/2025 - (616*t2)/405 + (1568*t)/81 - 7808/81
            self.y_d_dot = (2*t4)/50625 - (44*t3)/10125 + (118*t2)/675 - (1232*t)/405 + 1568/81
            self.y_d_ddot = (8*t3)/50625 - (44*t2)/3375 + (236*t)/675 - 1232/405
        
        elif t<50:
            self.z_d = 1
            self.z_d_dot = 0
            self.z_d_ddot = 0
            t2 = t*t
            t3 = t2*t
            t4 = t3*t
            t5 = t4*t

            self.x_d = - (2*t5)/253125 + (17*t4)/10125 - (286*t3)/2025 + (476*t2)/81 - (9800*t)/81 + 80000/81
            self.x_d_dot = - (2*t4)/50625 + (68*t3)/10125 - (286*t2)/675 + (952*t)/81 - 9800/81
            self.x_d_ddot = - (8*t3)/50625 + (68*t2)/3375 - (572*t)/675 + 952/81

            self.y_d = 1
            self.y_d_dot = 0
            self.y_d_ddot = 0

        elif t<65:
            self.z_d = 1
            self.z_d_dot = 0
            self.z_d_ddot = 0
            t2 = t*t
            t3 = t2*t
            t4 = t3*t
            t5 = t4*t

            self.x_d = 0
            self.x_d_dot = 0
            self.x_d_ddot = 0

            self.y_d = - (2*t5)/253125 + (23*t4)/10125 - (526*t3)/2025 + (1196*t2)/81 - (33800*t)/81 + 380081/81
            self.y_d_dot = - (2*t4)/50625 + (92*t3)/10125 - (526*t2)/675 + (2392*t)/81 - 33800/81
            self.y_d_ddot = - (8*t3)/50625 + (92*t2)/3375 - (1052*t)/675 + 2392/81


    def smc_control(self, xyz, xyz_dot, rpy, rpy_dot):
        m = 27*1e-3
        l = 46*1e-3
        Ix = 16.571710*1e-6
        Iy = 16.571710*1e-6
        Iz = 29.261652*1e-6
        Ip = 12.65625*1e-8
        kf = 1.28192*1e-8
        km = 5.964552*1e-3
        g = 9.81
        lmbda = [5,15,13,5]
        k = [13,180,150,25]
        pid = [80,10,80,10]


        

        all_mat = np.array([[1/(4*kf), -np.sqrt(2)/(4*kf*l),-np.sqrt(2)/(4*kf*l), -1/(4*km*kf)],
                   [1/(4*kf), -np.sqrt(2)/(4*kf*l), np.sqrt(2)/(4*kf*l),  1/(4*km*kf)],
                   [1/(4*kf),  np.sqrt(2)/(4*kf*l), np.sqrt(2)/(4*kf*l), -1/(4*km*kf)],
                   [1/(4*kf),  np.sqrt(2)/(4*kf*l), -np.sqrt(2)/(4*kf*l), 1/(4*km*kf)]])

        # obtain the desired values by evaluating the corresponding trajectories
        self.traj_evaluate()
        # TODO: implement the Sliding Mode Control laws designed in Part 2 to calculate the control inputs "u"
        # REMARK: wrap the roll-pitch-yaw angle errors to [-pi to pi]
        # TODO: convert the desired control inputs "u" to desired rotor velocities "motor_vel" by using the "allocation matrix"
        # TODO: maintain the rotor velocities within the valid range of [0 to 2618]
        # publish the motor velocities to the associated ROS topic

        fx = m*(-pid[0]*(xyz[0][0]-self.x_d) - pid[1]*(xyz_dot[0][0] - self.x_d_dot) + self.x_d_ddot)
        fy = m*(-pid[2]*(xyz[1][0]-self.y_d) - pid[3]*(xyz_dot[1][0] - self.y_d_dot) + self.y_d_ddot)

        e1 = self.z_d - xyz[2,0]
        e1_dot = self.z_d_dot - xyz_dot[2,0]
        s1 = e1_dot + lmbda[0]*e1

        u1 = (m/(np.cos(rpy[0,0])*np.cos(rpy[1,0])))*(self.z_d_ddot + g + lmbda[0]*e1_dot + k[0]*self.sat(s1,0.9))
        if u1!=0:
            theta_d = np.arcsin(np.clip(fx/u1,-1,1))
            phi_d = np.arcsin(np.clip(-fy/u1,-1,1))
        else:
            theta_d = 0
            phi_d = 0
        
        e2_dot = self.phi_d_dot - rpy_dot[0,0]
        e3_dot = self.theta_d_dot - rpy_dot[1,0]
        e4_dot = self.psi_d_dot - rpy_dot[2,0]

        
        e2 = phi_d - rpy[0,0]
        if e2 > pi:
            e2 = pi
        elif e2 < -pi:
            e2 = -pi

        e3 = theta_d - rpy[1,0]
        if e3 > pi:
            e3 = pi
        elif e3 < -pi:
            e3 = -pi

        e4 = self.psi_d - rpy[2,0]
        if e4 > pi:
            e4 = pi
        elif e4 < -pi:
            e4 = -pi

        
        s2 = e2_dot + lmbda[1]*e2
        s3 = e3_dot + lmbda[2]*e3
        s4 = e4_dot + lmbda[3]*e4

        
        u2 = Ix*self.phi_d_ddot - rpy_dot[1,0]*rpy_dot[2,0]*(Iy-Iz) + Ip*self.omega*rpy_dot[1,0] + Ix*lmbda[1]*e2_dot + Ix*k[1]*self.sat(s2,0.9)
        u3 = Iy*self.theta_d_ddot - rpy_dot[0,0]*rpy_dot[2,0]*(Iz-Ix) - Ip*self.omega*rpy_dot[0,0] + Iy*lmbda[2]*e3_dot + Iy*k[2]*self.sat(s3,0.9)
        u4 = Iz*self.psi_d_ddot - rpy_dot[0,0]*rpy_dot[2,0]*(Ix-Iy) + Iz*lmbda[3]*e4_dot + Iz*k[3]*self.sat(s4,0.9)
        
        u = np.array([[u1,u2,u3,u4]],dtype = object).T

        motor_vel = np.dot(all_mat,u)

        if motor_vel[0,0]>0:
            motor_vel[0,0] = np.sqrt(motor_vel[0,0])
        else:
            motor_vel[0,0]=0
        
        if motor_vel[1,0]>0:
            motor_vel[1,0] = np.sqrt(motor_vel[1,0])
        else:
            motor_vel[1,0]=0
        
        if motor_vel[2,0]>0:
            motor_vel[2,0] = np.sqrt(motor_vel[2,0])
        else:
            motor_vel[2,0]=0
        
        if motor_vel[3,0]>0:
            motor_vel[3,0] = np.sqrt(motor_vel[3,0])
        else:
            motor_vel[3,0]=0


        # motor_vel[0,0] = np.max(0,np.min(motor_vel[0,0],2618))
        # motor_vel[1,0] = np.max(0,np.min(motor_vel[1,0],2618))
        # motor_vel[2,0] = np.max(0,np.min(motor_vel[2,0],2618))
        # motor_vel[3,0] = np.max(0,np.min(motor_vel[3,0],2618))

        motor_vel[0,0] = np.clip(motor_vel[0,0], 0, 2618)
        motor_vel[1,0] = np.clip(motor_vel[1,0], 0, 2618)
        motor_vel[2,0] = np.clip(motor_vel[2,0], 0, 2618)
        motor_vel[3,0] = np.clip(motor_vel[3,0], 0, 2618)


        motor_speed = Actuators()
        motor_speed.angular_velocities = [motor_vel[0,0], motor_vel[1,0], motor_vel[2,0], motor_vel[3,0]]
        self.motor_speed_pub.publish(motor_speed)
        self.omega = u1 - u2 + u3 - u4
        

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
        with open("/home/mayank/rbe502_project/log.pkl","wb") as fp:
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
        
    