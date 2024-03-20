import matplotlib.pyplot as plt
import numpy as np
from random import gauss
from pygame.locals import *
import pygame
import sys

class Micromouse():
    def __init__(self):
        #MicroMouse parameter (SI units)
        self.gravity = 9.81
        self.robot_mass   = 0.1
        self.robot_width  = 0.08
        self.robot_length = 0.1
        self.robot_cg     = 0.024
        self.tier_width   = 0.008
        self.tier_r       = 0.012
        self.tier_mass    = 0.005
        self.Jmot         = 0.5 * self.tier_mass * self.tier_r**2
        self.mu           = 0.03
        self.Ct           = 1.0
        self.Ctr          = self.Ct
        self.Ctl          = self.Ct
        self.Cc           = 1.0
        self.motor_R      = 1.07
        self.motor_K      = 1.98e-3
        self.gear         = 10.0
        self.tread = self.robot_width - self.tier_width
        self.robot_inertia =  self.robot_mass*(self.robot_width**2 + self.robot_length**2)/3.0
        self.time = 0.0
        self.steptime = 1e-4
        self.nrflag = False
        self.nlflag = False
        self.ground_noise_flag = 0.0

    def __rk4(self, func, t, h, y, *x):
        '''
        @brief 4次のルンゲ・クッタ法を一回分計算する関数

        @details この関数では時刻は更新されないため、これとは別に時間更新をする必要があります。

        :parame func:導関数
        :parame t:現在時刻を表す変数
        :parame h:刻み幅
        :parame y:出力変数（求めたい値）
        :parame *x:引数の数が可変する事に対応する、その他の必要変数
        :return 1step分の結果を出力
        '''
        k1=h*func(t, y, *x)
        k2=h*func(t+0.5*h, y+0.5*k1, *x)
        k3=h*func(t+0.5*h, y+0.5*k2, *x) 
        k4=h*func(t+h, y+k3, *x)
        y=y+(k1 + 2*k2 + 2*k3 + k4)/6
        return y

    '''
    導関数の書き方
    def func(t, y, *state):
        func:自分で好きな関数名をつけられます
        t:時刻変数(変数の文字はtで無くても良い) 
        y:出力変数(変数の文字はyで無くても良い)
        *state:その他の必要変数(引数の数は可変可能))
    #関数サンプル
    def vdot(t, y, *state):
        s1=state[0]
        s2=state[1]
        return t+y+s1+s2    
    '''

    def __drvforce(self, omega, velocity, N):
        #タイヤの推進力
        cirspeed = self.tier_r * omega/self.gear
        lam = (cirspeed - velocity)/max(abs(velocity),abs(cirspeed),0.0001)
        return self.Ct * lam * N

    def __drvforce_r(self, omega, velocity, N):
        #タイヤの推進力
        cirspeed = self.tier_r * omega/self.gear
        lam = (cirspeed - velocity)/max(abs(velocity),abs(cirspeed),0.0001)
        return self.Ctr * lam * N

    def __drvforce_l(self, omega, velocity, N):
        #タイヤの推進力
        cirspeed = self.tier_r * omega/self.gear
        lam = (cirspeed - velocity)/max(abs(velocity),abs(cirspeed),0.0001)
        return self.Ctl * lam * N

    def __cforce(self, beta, N):
        #タイヤの横力
        return self.Cc * beta * N

    def __Nforce_r(self, u, r):
        #右のタイヤの垂直抗力 
        nr = self.robot_mass * (0.5*self.gravity + self.robot_cg * u * r/self.tread)
        if nr <0 or nr > self.robot_mass * self.gravity:
            self.nrflag = True
        return nr

    def __Nforce_l(self, u, r):
        #左のタイヤの垂直抗力
        nl = self.robot_mass * (0.5*self.gravity - self.robot_cg * u * r/self.tread)
        if nl < 0 or nl > self.robot_mass * self.gravity:
            self.nlflag = True
        return nl

    def __omegadot_r(self, t, omega, *state):
        '''
        @brief 右のタイヤの角加速度
        :parame: u
        :parame: v
        :parame: voltage
        '''
        u = state[0]
        v = state[1]
        r = state[2]
        voltage = state[3]
        i = (voltage - self.motor_K * omega)/self.motor_R 
        T = self.motor_K * i
        beta = np.arctan2(v, u)
        NR = self.__Nforce_r(u, r)
        f = self.__drvforce_r(omega, u + r*self.tread/2, NR)
        return (T- self.tier_r * f/self.gear)/self.Jmot

    def __omegadot_l(self, t, omega, *state):
        '''
        @brief 左のタイヤの角加速度
        :parame: u
        :parame: v
        :parame: voltage
        '''
        u = state[0]
        v = state[1]
        r = state[2]
        voltage = state[3]
        i = (voltage - self.motor_K * omega)/self.motor_R 
        T = self.motor_K * i
        beta = np.arctan2(v, u)
        NL = self.__Nforce_l(u, r)
        f = self.__drvforce_l(omega, u-r*self.tread/2, NL)
        return (T- self.tier_r * f/self.gear)/self.Jmot

    def __udot(self, t, u, *state):
        '''
        @brief 機体軸縦速度
        :parame: omega_r
        :parame: omega_l
        :parame: v
        :parame: r
        '''
        omega_r = state[0]
        omega_l = state[1]
        v = state[2]
        r = state[3]
        beta = np.arctan2(v, u)
        NR = self.__Nforce_r(u , r)
        fr = self.__drvforce_r(omega_r, u + r*self.tread/2, NR)
        NL = self.__Nforce_l(u, r)
        fl = self.__drvforce_l(omega_l, u - r*self.tread/2, NL)
        return r * v + (fr + fl)/self.robot_mass

    def __vdot(self, t, v, *state):
        '''
        @brief 機体軸横速度
        :parame: u
        :parame: r
        '''
        u = state[0]
        r = state[1]
        beta_r = np.arctan2(v, u + r * self.tread/2)
        NR = self.__Nforce_r(u, r)
        fcr = -self.__cforce(beta_r, NR)
        beta_l = np.arctan2(v, u - r * self.tread/2)
        NL = self.__Nforce_l(u, r)
        fcl = -self.__cforce(beta_l, NL)
        #print(beta_l)
        return -r * u + (fcr+fcl)/self.robot_mass
    
    def __rdot(self, t, r, *state):
        '''
        @brief ヨー角加速度
        :parame: omega_r
        :parame: omega_l
        :parame: u
        :parame: v
        '''
        omega_r = state[0]
        omega_l = state[1]
        u = state[2]
        v = state[3]
        beta = np.arctan2(v, u)
        NR = self.__Nforce_r(u, r)
        fr = self.__drvforce_r(omega_r, u + r*self.tread/2 , NR)
        NL = self.__Nforce_l(u, r)
        fl = self.__drvforce_l(omega_l, u - r*self.tread/2, NL)
        return self.tread*(fr-fl)/2.0/self.robot_inertia

    def __psidot(self, t, psi, *state):
        '''
        @brief 迷路軸角速度
        :parame: r
        '''
        r = state[0]
        return r

    def __Xdot(self, t, X, *state):
        '''
        @brief 迷路軸東西加速度
        :parame: u
        :parame: v
        :parame: psi
        '''
        #
        u = state[0]
        v = state[1]
        psi = state[2]
        return np.cos(psi)*u - np.sin(psi)*v
    
    def __Ydot(self, t, Y, *state):
        '''
        @brief 迷路軸南北加速度
        :parame: u
        :parame: v
        :parame: psi
        '''
        u = state[0]
        v = state[1]
        psi = state[2]
        return np.sin(psi)*u + np.cos(psi)*v
    
    def ground_noise_on(self):
        self.ground_noise_flag = 1.0

    def ground_noise_off(self):
        self.ground_noise_flag = 0.0

    def version(self):
        print("-----------------------")
        print(" Micromouse Simulator ")
        print(" Version 1.0")
        print("-----------------------")
              
    def step(self, t, input_r, input_l, state):
        self.Ctr = self.Ct + self.ground_noise_flag*gauss(0, 1)
        self.Ctl = self.Ct + self.ground_noise_flag*gauss(0, 1)
        omega_r =state[0]
        omega_l =state[1]
        u = state[2]
        v = state[3]
        r = state[4]
        psi = state[5]
        X = state[6]
        Y = state[7]
        state[0] = self.__rk4(self.__omegadot_r, self.time, self.steptime, omega_r, u, v, r, input_r)
        state[1] = self.__rk4(self.__omegadot_l, self.time, self.steptime, omega_l, u, v, r, input_l)
        state[2] = self.__rk4(self.__udot, self.time, self.steptime, u, omega_r, omega_l, v, r)
        state[3] = self.__rk4(self.__vdot, self.time, self.steptime, v, u, r)
        state[4] = self.__rk4(self.__rdot, self.time, self.steptime, r, omega_r, omega_l, u, v)
        state[5] = self.__rk4(self.__psidot, self.time, self.steptime, psi, r)
        state[6] = self.__rk4(self.__Xdot, self.time, self.steptime, X, u, v, psi)
        state[7] = self.__rk4(self.__Ydot, self.time, self.steptime, Y, u, v, psi)
        self.time += self.steptime 
        return self.time, state

if __name__ == "__main__":
    mouse = Micromouse()
    mouse.version()