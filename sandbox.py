from  micromouse import Micromouse
from maze import Maze
import matplotlib.pyplot as plt
import numpy as np
from random import gauss
from pygame.locals import *
import pygame
import sys


def main():
    pygame.init()    # Pygameを初期化
    clock = pygame.time.Clock()
    screen = pygame.display.set_mode((640, 640))    # 画面を作成
    pygame.display.set_caption("Micromouse Simulation")    # タイトルを作成
    FPS = 24

    mouse = Micromouse()
    maze = Maze(0)
    mouse.ground_noise_on()

    while True:
        time =0.0
        mouse.time = time
        drawtime = 0.0
        state=[0,0,0,0,0,np.pi/2,0.09,0.09]
        mouse.state = state
        ur = 0.0
        ul = 0.0

        while time<3.5 and 0.04<state[6]<0.14:
            if  state[7]<(180*14-50)/1000:
                ur = 2.0
                ul = 2.0
            else:
                ur = 0.0
                ul = 0.0
            time, state= mouse.step(time, ur, ul)
            #print(time, state[6], state[7], ur, ul)

            if time >= drawtime:
                drawtime += 1/FPS
                maze.draw_map(screen)
                mouse.draw_robot(screen)
                pygame.display.update() #描画処理を実行

            for event in pygame.event.get():
                if event.type == QUIT:  # 終了イベント
                    pygame.quit()  #pygameのウィンドウを閉じる
                    #sys.exit() #システム終了
                    return

if __name__ == "__main__":
    main() 
