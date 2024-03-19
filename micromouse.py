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

def main():
    pygame.init()    # Pygameを初期化
    clock = pygame.time.Clock()
    screen = pygame.display.set_mode((320, 240))    # 画面を作成
    pygame.display.set_caption("Micromouse Simulation")    # タイトルを作成
    FPS = 30
    #---------------  1.画像を読み込む  --------------------------
    img_width = 64
    img_height = 46

    #一部の色を透明にする
    zoom =3.0
    img0 = pygame.image.load("micromouse.png").convert()
    #print(img2.get_size())
    #colorkey = (255, 255, 255)#img2.get_at((207,0))
    #img2.set_colorkey(colorkey, RLEACCEL)
    rotation = 90
    running = True
    flag = False
    render_time = 0.0
    render_step = 1/FPS


    state=[0,0,0,0,0,0.0,0.09,0.09]
    mouse=Micromouse()
    time = 0.0
    Time=[]
    Time2=[]
    State=[[],[],[],[],[],[],[],[]]
    State_input=[[],[]]
    State_ref=[[],[]]
    control_time = 0.0
    control_step = 1e-3
    control_counter = 0
    ref_r = 0.0
    ref_l = 0.0
    ref_psi_final = 90*np.pi/180
    ref_psi = 0.0
    sum_r = 0.0
    sum_l = 0.0
    sum_err = 0.0
    omega0 = 0
    omega = 0.0
    deltaT = 0.3
    omega_max = ref_psi_final/2/deltaT
    angle_acc = omega_max/deltaT*1.00434821
    psi = 0.0
    psi0 = 0.0
    psi1 = 0.0
    u_r = 0.0
    u_l = 0.0
    ref_omega = 0.0
    omega_err = 0.0
    old_omega_err = 0.0
    sum_omega_err = 0.0
    old_err_psi = 0.0
    err_psi = 0.0
    offset = 0.5
    offset_count = int(offset/control_step)
    print(offset_count)
    time_max = 2.5


    for _ in range(int(time_max/mouse.steptime)):
        if mouse.nlflag or mouse.nrflag:
            break
        Time.append(time)        
        for i in range(8):
            State[i].append(state[i])
        #print(state)
        
        if control_time<=time:
            Time2.append(time)
            State_ref[0].append(ref_psi)
            State_ref[1].append(ref_omega)
            State_input[0].append(u_r)
            State_input[1].append(u_l)

            control_time += control_step
            omega_r = state[0]
            omega_l = state[1]
            omega = state[4]
            psi = state[5]

            if control_counter < offset_count:
                ref_psi = 0.0
            elif control_counter < 300+offset_count:
                omega0 = angle_acc * (time-offset)
                ref_psi = 0.5*angle_acc * (time-offset)**2
                psi0 = ref_psi
            elif control_counter < 600+offset_count:
                ref_psi = psi0 + omega0 * (time -offset - deltaT + 1*control_step)
                psi1 = ref_psi
            elif control_counter < 899+offset_count:
                ref_psi = psi1 + omega0 * (time-offset -2*deltaT + 1*control_step) - 0.5* angle_acc * (time -offset -2*deltaT + 1*control_step)**2 
            else:
                ref_psi = ref_psi
            
            kp = 150.0
            ki = 1000
            kd = 0.3#0.7

            kpo = 1.0
            kio= 20#40
            kdo = 0.025#0.03

            old_err_psi = err_psi
            err_psi = ref_psi - psi
            sum_err = sum_err + err_psi * control_step
            diff_err_psi = (err_psi - old_err_psi)/control_step
            ref_omega = kp * err_psi + ki * sum_err + kd * diff_err_psi

    
            old_omega_err = omega_err
            omega_err = ref_omega - omega
            sum_omega_err = sum_omega_err + omega_err * control_step
            def_omega_err = (omega_err - old_omega_err)/control_step
            u_omega = kpo * omega_err + kio*sum_omega_err + kdo * def_omega_err 

            u_r = u_omega/2
            u_l =-u_omega/2 
 
            control_counter +=1

        time, state = mouse.step(time, u_r, u_l, state)

        #Rendering
        rotation = psi * 180/np.pi
        if time>render_time:
            render_time += render_step
            screen.fill((0,0,0))  #画面を黒で塗りつぶす
            img = pygame.transform.scale(img0, (img_width*zoom, img_height*zoom)) 
            rotated_image = pygame.transform.rotate(img, rotation)
            new_rect = rotated_image.get_rect(center=img.get_rect(center=(160, 120)).center)
            screen.blit(rotated_image, new_rect)

            pygame.display.update() #描画処理を実行
            clock.tick(FPS)

        for event in pygame.event.get():
            if event.type == QUIT:  # 終了イベント
                pygame.quit()  #pygameのウィンドウを閉じる
                sys.exit() #システム終了
    #End of Loop
    #pygame.quit()  #pygameのウィンドウを閉じる
    #print(ref_psi*180/np.pi)
    
    plt.figure("State")
    min_y=[-100,-10,-1,-1,-5,-2.5,0,0]
    max_y=[10,100, 1, 1, 5, 1, 0.18, 0.18]
    for i in range(8):
        plt.subplot(8,1,i+1)
        plt.plot(Time, State[i], label="data{}".format(i))
        #plt.ylim(min_y[i], max_y[i])
        plt.legend()
        plt.grid()

    '''
    plt.plot(State[6], State[7])
    plt.xlim(0, 1.8)
    plt.ylim(0, 1.8)
    plt.xticks(np.arange(0, 1.81, 0.18))
    plt.yticks(np.arange(0, 1.81, 0.18))       
    plt.grid()
    plt.show()
    '''

    plt.figure("Control result")
    plt.subplot(4,1,1)
    #plt.plot(Time, State[0])
    plt.plot(Time2, State_input[0])
    plt.grid()
    plt.xlim(0, time_max)
    plt.ylabel("Input(R)[V]")
    plt.subplot(4,1,2)
    #plt.plot(Time, State[1])
    plt.plot(Time2, State_input[1])
    plt.ylabel("Input(L)[V]")
    plt.grid()
    plt.xlim(0, time_max)
    #plt.show()
    plt.subplot(4,1,3)
    plt.plot(Time, np.array(State[5])*180/np.pi)
    plt.plot(Time2, np.array(State_ref[0])*180/np.pi)
    plt.yticks((0, 45, 90))
    plt.xlim(0, time_max)
    plt.ylabel("Angle[deg]")
    plt.grid()

    plt.subplot(4,1,4)
    plt.plot(Time, np.array(State[4])*180/np.pi)
    plt.plot(Time2, np.array(State_ref[1])*180/np.pi)
    plt.ylabel("Angle vel[deg/s]")
    plt.xlabel("Time[s]")
    plt.xlim(0, time_max)

    plt.grid()
    plt.show()

def main2():
    pygame.init()    # Pygameを初期化
    clock = pygame.time.Clock()
    screen = pygame.display.set_mode((320, 240))    # 画面を作成
    pygame.display.set_caption("Micromouse Simulation")    # タイトルを作成
    FPS = 30
    #---------------  1.画像を読み込む  --------------------------
    img_width = 64
    img_height = 46

    #一部の色を透明にする
    zoom =3.0
    img0 = pygame.image.load("micromouse.png").convert()
    #print(img2.get_size())
    #colorkey = (255, 255, 255)#img2.get_at((207,0))
    #img2.set_colorkey(colorkey, RLEACCEL)
    rotation = 90
    running = True
    flag = False
    render_time = 0.0
    render_step = 1/FPS


    state=[0,0,0,0,0,0.0,0.09,0.09]
    mouse=Micromouse()
    time = 0.0
    Time=[]
    Time2=[]
    State=[[],[],[],[],[],[],[],[]]
    State_input=[[],[]]
    State_ref=[[],[]]
    control_time = 0.0
    control_step = 1e-3
    control_counter = 0
    ref_r = 0.0
    ref_l = 0.0
    ref_psi_final = 90*np.pi/180
    ref_psi = 0.0
    sum_r = 0.0
    sum_l = 0.0
    sum_err = 0.0
    omega0 = 0
    omega = 0.0
    deltaT = 0.3
    omega_max = ref_psi_final/2/deltaT
    angle_acc = omega_max/deltaT*1.00434821
    psi = 0.0
    psi0 = 0.0
    psi1 = 0.0
    u_r = 0.0
    u_l = 0.0
    ref_omega = 0.0
    omega_err = 0.0
    old_omega_err = 0.0
    sum_omega_err = 0.0
    old_err_psi = 0.0
    err_psi = 0.0
    offset = 0.5
    offset_count = int(offset/control_step)
    print(offset_count)
    time_max = 2.5


    for _ in range(int(time_max/mouse.steptime)):
        if mouse.nlflag or mouse.nrflag:
            break
        Time.append(time)        
        for i in range(8):
            State[i].append(state[i])
        #print(state)
        
        if control_time<=time:
            Time2.append(time)
            State_ref[0].append(ref_psi)
            State_ref[1].append(ref_omega)
            State_input[0].append(u_r)
            State_input[1].append(u_l)

            control_time += control_step
            omega_r = state[0]
            omega_l = state[1]
            omega = state[4]
            psi = state[5]

            ref_psi = np.pi*0.5/(1+np.exp(-10*(time - offset-0.45)))
            
            kp = 150.0
            ki = 1000
            kd = 0.3#0.7

            kpo = 1.0
            kio= 20#40
            kdo = 0.025#0.03

            old_err_psi = err_psi
            err_psi = ref_psi - psi
            sum_err = sum_err + err_psi * control_step
            diff_err_psi = (err_psi - old_err_psi)/control_step
            ref_omega = kp * err_psi + ki * sum_err + kd * diff_err_psi

    
            old_omega_err = omega_err
            omega_err = ref_omega - omega
            sum_omega_err = sum_omega_err + omega_err * control_step
            def_omega_err = (omega_err - old_omega_err)/control_step
            u_omega = kpo * omega_err + kio*sum_omega_err + kdo * def_omega_err 

            u_r = u_omega/2
            u_l =-u_omega/2 
 
            control_counter +=1

        time, state = mouse.step(time, u_r, u_l, state)

        #Rendering
        rotation = psi * 180/np.pi
        if time>render_time:
            render_time += render_step
            screen.fill((0,0,0))  #画面を黒で塗りつぶす
            img = pygame.transform.scale(img0, (img_width*zoom, img_height*zoom)) 
            rotated_image = pygame.transform.rotate(img, rotation)
            new_rect = rotated_image.get_rect(center=img.get_rect(center=(160, 120)).center)
            screen.blit(rotated_image, new_rect)

            pygame.display.update() #描画処理を実行
            clock.tick(FPS)

        for event in pygame.event.get():
            if event.type == QUIT:  # 終了イベント
                pygame.quit()  #pygameのウィンドウを閉じる
                sys.exit() #システム終了
    #End of Loop
    #pygame.quit()  #pygameのウィンドウを閉じる
    #print(ref_psi*180/np.pi)
    
    plt.figure("State")
    min_y=[-100,-10,-1,-1,-5,-2.5,0,0]
    max_y=[10,100, 1, 1, 5, 1, 0.18, 0.18]
    for i in range(8):
        plt.subplot(8,1,i+1)
        plt.plot(Time, State[i], label="data{}".format(i))
        #plt.ylim(min_y[i], max_y[i])
        plt.legend()
        plt.grid()

    '''
    plt.plot(State[6], State[7])
    plt.xlim(0, 1.8)
    plt.ylim(0, 1.8)
    plt.xticks(np.arange(0, 1.81, 0.18))
    plt.yticks(np.arange(0, 1.81, 0.18))       
    plt.grid()
    plt.show()
    '''

    plt.figure("Control result")
    plt.subplot(4,1,1)
    #plt.plot(Time, State[0])
    plt.plot(Time2, State_input[0])
    plt.grid()
    plt.xlim(0, time_max)
    plt.ylabel("Input(R)[V]")
    plt.subplot(4,1,2)
    #plt.plot(Time, State[1])
    plt.plot(Time2, State_input[1])
    plt.ylabel("Input(L)[V]")
    plt.grid()
    plt.xlim(0, time_max)
    #plt.show()
    plt.subplot(4,1,3)
    plt.plot(Time, np.array(State[5])*180/np.pi)
    plt.plot(Time2, np.array(State_ref[0])*180/np.pi)
    plt.yticks((0, 45, 90))
    plt.xlim(0, time_max)
    plt.ylabel("Angle[deg]")
    plt.grid()

    plt.subplot(4,1,4)
    plt.plot(Time, np.array(State[4])*180/np.pi)
    plt.plot(Time2, np.array(State_ref[1])*180/np.pi)
    plt.ylabel("Angle vel[deg/s]")
    plt.xlabel("Time[s]")
    plt.xlim(0, time_max)

    plt.grid()
    plt.show()



if __name__ == "__main__":
    main2()