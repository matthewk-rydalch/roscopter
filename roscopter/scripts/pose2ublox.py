#!/usr/bin/env python3

import numpy as np


class Pose2Ublox():
    

    def __init__(self, Ts, gha, gva, gsa, rha, rva, rsa, cha, no, rl, srp, srv, srr, sbp, sbv, lo, A, B):

        #parameters
        self.Ts = Ts
        self.global_horizontal_accuracy = gha
        self.global_vertical_accuracy = gva
        self.global_speed_accuracy = gsa
        self.relative_horizontal_accuracy = rha
        self.relative_vertical_accuracy = rva
        self.relative_speed_accuracy = rsa
        self.compassing_heading_accuracy = cha
        self.noise_on = no
        self.ref_lla = rl
        self.sigma_rover_pos = srp 
        self.sigma_rover_vel = srv
        self.sigma_rover_relpos = srr
        self.sigma_base_pos = sbp
        self.sigma_base_vel = sbv
        self.lpf_on = lo
        self.A = A
        self.B = B
        self.Kgps = 1100
        self.eta_h = 0.03
        self.eta_a = 0.06

        #calc ref_ecef from ref_lla
        self.F = (self.A - self.B)/self.A     # Ellipsoid Flatness
        self.E2 = self.F * (2.0 - self.F);    # Square of Eccentricity    
        self.ref_ecef = self.lla2ecef(self.ref_lla)

        #combine standard deviations
        self.ned_std_dev_3d = [self.global_horizontal_accuracy, self.global_horizontal_accuracy, self.global_vertical_accuracy]
        self.vel_std_dev_3d = [self.global_speed_accuracy, self.global_speed_accuracy, self.global_speed_accuracy]
        self.relpos_std_dev_3d = [self.relative_horizontal_accuracy, self.relative_horizontal_accuracy, self.relative_vertical_accuracy]
        self.white_noise_3d = [self.eta_h, self.eta_h, self.eta_a]

        #other needed variables and arrays
        self.rover_ned = np.zeros(3)
        self.rover_ned_prev = np.zeros(3)
        self.rover_ned_lpf = np.zeros(3)
        self.rover_vel_lpf = np.zeros(3)
        self.rover_prev_time = 0.0
        self.rover_relpos_lpf = np.zeros(3)
        self.base_ned = np.zeros(3)
        self.base_ned_prev = np.zeros(3)
        self.base_ned_lpf = np.zeros(3)
        self.base_vel_lpf = np.zeros(3)
        self.base2_quat = np.zeros(4)
        self.base_prev_time = 0.0
        self.rover_ned_noise = np.zeros(3)
        self.rover_vel_noise = np.zeros(3)
        self.rover_vel_noise_prev = np.zeros(3)
        self.base_ned_noise = np.zeros(3)
        self.base_vel_noise = np.zeros(3)
        self.base_vel_noise_prev = np.zeros(3)

        #Outputs
        self.rover_virtual_pos_ecef = np.zeros(3)
        self.rover_virtual_vel_ecef = np.zeros(3)
        self.rover_virtual_relpos = np.zeros(3)
        self.base_virtual_pos_ecef = np.zeros(3)
        self.base_virtual_vel_ecef = np.zeros(3)
        self.base2_heading = 0.0


    def update_rover_virtual_PosVelEcef(self, dt):

        #calculate virtual position in ecef frame with noise
        self.rover_ned_noise = self.add_gps_noise(self.white_noise_3d, self.rover_ned_noise, self.sigma_rover_pos)
        rover_ned_w_noise = self.rover_ned + self.rover_ned_noise 
        virtual_delta_ecef = self.ned2ecef(rover_ned_w_noise, self.ref_lla)
        self.rover_virtual_pos_ecef = self.ref_ecef + virtual_delta_ecef

        #calculate virtual velocity in ecef frame with noise
        #make sure we do not divide by zero
        if dt != 0.0:
            rover_vel = (self.rover_ned - self.rover_ned_prev)/dt
        else:
            rover_vel = np.zeros(3)
        self.rover_vel_noise = self.add_noise_3d(np.zeros(3), self.white_noise_3d)
        self.rover_vel_lpf = self.lpf(self.rover_vel_noise, self.rover_vel_noise_prev, self.Ts, self.sigma_rover_vel)
        rover_vel_w_noise = rover_vel + self.rover_vel_lpf
        self.rover_virtual_vel_ecef = self.ned2ecef(rover_vel_w_noise, self.ref_lla)

        #update histories
        self.rover_ned_prev = self.rover_ned
        self.rover_vel_prev = rover_vel
        self.rover_vel_noise_prev = self.rover_vel_noise



    def update_rover_virtual_relPos(self):

        #calculate virtual relative position of the rover with respect to the base in ned frame with noise.
        relpos_array = self.rover_ned - self.base_ned
        rover_relpos_noise = self.add_noise_3d(relpos_array, self.relpos_std_dev_3d)
        self.rover_relpos_lpf = self.lpf(rover_relpos_noise, self.rover_relpos_lpf, self.Ts, self.sigma_rover_relpos)

        self.rover_virtual_relpos = self.rover_relpos_lpf


    def update_base_virtual_PosVelEcef(self, dt):

        #calculate virtual position in ecef frame with noise
        self.base_ned_noise = self.add_gps_noise(self.white_noise_3d, self.base_ned_noise, self.sigma_base_pos)
        base_ned_w_noise = self.base_ned + self.base_ned_noise
        virtual_delta_ecef = self.ned2ecef(base_ned_w_noise, self.ref_lla)
        self.base_virtual_pos_ecef = self.ref_ecef + virtual_delta_ecef

        #calculate virtual velocity in ecef frame with noise
        #make sure we do not divide by zero
        if dt != 0.0:
            base_vel = (self.base_ned - self.base_ned_prev)/dt
        else:
            base_vel = np.zeros(3)
        self.base_vel_noise = self.add_noise_3d(np.zeros(3), self.white_noise_3d)
        self.base_vel_lpf = self.lpf(self.base_vel_noise, self.base_vel_noise_prev, self.Ts, self.sigma_base_vel)
        base_vel_w_noise = base_vel + self.base_vel_lpf
        self.base_virtual_vel_ecef = self.ned2ecef(base_vel_w_noise, self.ref_lla)

        #update histories
        self.base_ned_prev = self.base_ned
        self.base_vel_prev = base_vel
        self.base_vel_noise_prev = self.base_vel_noise

    def update_base2_virtual_relPos(self):

        euler = self.quat2euler(self.base2_quat)
        self.base2_heading = euler[2]
        if self.noise_on:
            self.base2_heading = np.random.normal(self.base2_heading, self.compassing_heading_accuracy)


    def add_noise_3d(self, value, std_dev):
        
        if self.noise_on:
            value_w_noise_1 = np.random.normal(value[0], std_dev[0])
            value_w_noise_2 = np.random.normal(value[1], std_dev[1])
            value_w_noise_3 = np.random.normal(value[2], std_dev[2])
            value_w_noise = np.array([value_w_noise_1, value_w_noise_2, value_w_noise_3])
            return value_w_noise

        else:
            return value


    def add_gps_noise(self, white_noise_3d, noise_prev, sigma):

        #Eq 7.17 Small Unmanned Aircraft (Beard, McLain) pg 139
        white_noise = self.add_noise_3d(np.zeros(3), self.white_noise_3d)
        color_noise = np.exp(-self.Kgps*self.Ts)*noise_prev
        total_noise = white_noise + color_noise
        noise = self.lpf(total_noise, noise_prev, self.Ts, sigma)

        return noise

    def lpf(self, xt, x_prev, dt, sigma):
        
        #low pass filter
        if self.lpf_on:
            x_lpf = xt*dt/(sigma+dt) + x_prev*sigma/(sigma+dt)
            return x_lpf
        
        else:
            return xt


    def lla2ecef(self, lla):

        lat = lla[0]*np.pi/180
        lon = lla[1]*np.pi/180
        alt = lla[2]
        sinp = np.sin(lat)
        cosp = np.cos(lat)
        sinl = np.sin(lon)
        cosl = np.cos(lon)
        e2 = self.E2
        v = self.A/np.sqrt(1.0-e2*sinp*sinp)

        ecef = np.zeros(3)
        ecef[0]=(v+alt)*cosp*cosl
        ecef[1]=(v+alt)*cosp*sinl
        ecef[2]=(v*(1.0-e2)+lla[2])*sinp

        return ecef


    def ned2ecef(self, ned, lla):
        
        lat = lla[0]
        lon = lla[1]

        #don't know why @ isn't working for matrix multiplication
        # ecef = Ry(90)Rx(-lon)Ry(lat)ned
        ecef = np.matmul(np.matmul(np.matmul(self.Ry(90.0),self.Rx(-lon)),self.Ry(lat)),ned)

        return ecef


    def Rx(self, theta):
        
        theta = theta*np.pi/180.0
        st = np.sin(theta)
        ct = np.cos(theta)
        rotx = np.array([[1.0, 0.0, 0.0], \
                        [0.0, ct, st], \
                        [0.0, -st, ct]])

        return rotx


    def Ry(self, theta):
        
        theta = theta*np.pi/180.0
        st = np.sin(theta)
        ct = np.cos(theta)
        roty = np.array([[ct, 0.0, -st], \
                        [0.0, 1.0, 0.0], \
                        [st, 0.0, ct]])

        return roty


    def Rz(self, theta):

        theta = theta*np.pi/180.0
        st = np.sin(theta)
        ct = np.cos(theta)
        rotz = np.array([[ct, st, 0.0], \
                        [-st, ct, 0.0], \
                        [0.0, 0.0, 1.0]])

        return rotz


    def quat2euler(self, quat):
        
        qw = quat[0]
        qx = quat[1]
        qy = quat[2]
        qz = quat[3]

        # roll (x-axis rotation)
        sinr_cosp = 2.0 * (qw * qx + qy * qz)
        cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        # pitch (y-axis rotation)
        sinp = 2.0 * (qw * qy - qz * qx)
        pitch = np.arcsin(sinp)
        if abs(sinp) >= 1:
            pitch = np.pi*np.sign(sinp) / 2.0 # use 90 degrees if out of range

        # yaw (z-axis rotation)
        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2 * (qy * qy + qz * qz)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        euler = [roll, pitch, yaw]

        return euler
        