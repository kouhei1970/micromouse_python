from  micromouse import Micromouse
import matplotlib.pyplot as plt
import numpy as np
from random import gauss
from pygame.locals import *
import pygame
import sys

def main():
    mouse=Micromouse()
    pygame.init()    # Pygameを初期化
    clock = pygame.time.Clock()
    screen = pygame.display.set_mode((320, 240))    # 画面を作成
    pygame.display.set_caption("Micromouse Simulator")    # タイトルを作成
    FPS = 30
    #---------------  1.画像を読み込む  --------------------------
    robot_width = 80
    robot_length = 100

    #一部の色を透明にする
    zoom =2
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
    mouse.state = state
    mouse.ground_noise_on()
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

        time, state = mouse.step(time, u_r, u_l)

        #Rendering
        rotation = psi * 180/np.pi
        if time>render_time:
            render_time += render_step
            screen.fill((0,0,0))  #画面を黒で塗りつぶす
            img = pygame.transform.scale(img0, (robot_length*zoom, robot_width*zoom)) 
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
    draw_graph(Time, Time2, time_max, State, State_input, State_ref)

stop = False
def on_close(event):
    global stop
    stop = True
    pass  # ここで終了イベントを受け取って渡す？

def draw_graph(Time, Time2, time_max, State, State_input, State_ref):
    pygame_flag =True
    
    plt.ion()
    fig1 = plt.figure("State")
    fig1.canvas.mpl_connect('close_event', on_close)
    min_y=[-100,-10,-1,-1,-5,-2.5,0,0]
    max_y=[10,100, 1, 1, 5, 1, 0.18, 0.18]
    for i in range(8):
        ax = fig1.add_subplot(8,1,i+1)
        ax.plot(Time, State[i], label="data{}".format(i))
        #plt.ylim(min_y[i], max_y[i])
        ax.legend()
        ax.grid()

    fig = plt.figure("Control result")
    fig.canvas.mpl_connect('close_event', on_close)
    ax1 = fig.add_subplot(4,1,1)
    ax2 = fig.add_subplot(4,1,2)
    ax3 = fig.add_subplot(4,1,3)
    ax4 = fig.add_subplot(4,1,4)
    ax1.plot(Time2, State_input[0])
    ax1.grid()
    ax1.set_xlim(0, time_max)
    ax1.set_ylabel("Input(R)[V]")
    ax2.plot(Time2, State_input[1])
    ax2.set_ylabel("Input(L)[V]")
    ax2.grid()
    ax2.set_xlim(0, time_max)
    ax3.plot(Time, np.array(State[5])*180/np.pi)
    ax3.plot(Time2, np.array(State_ref[0])*180/np.pi)
    ax3.set_yticks((0, 45, 90))
    ax3.set_xlim(0, time_max)
    ax3.set_ylabel("Angle[deg]")
    ax3.grid()
    ax4.plot(Time, np.array(State[4])*180/np.pi)
    ax4.plot(Time2, np.array(State_ref[1])*180/np.pi)
    ax4.set_ylabel("Angle vel[deg/s]")
    ax4.set_xlabel("Time[s]")
    ax4.set_xlim(0, time_max)
    ax4.grid()

    while True:
        if stop == True:
            break
        fig1.canvas.flush_events()
        fig.canvas.flush_events()

        if pygame_flag == True:
            #pygame.display.update() #描画処理を実行
            for event in pygame.event.get():
                if event.type == QUIT:  # 終了イベント
                    pygame.quit()  #pygameのウィンドウを閉じる
                    pygame_flag = False
                    sys.exit() #システム終了
        
if __name__ == "__main__":
    main()