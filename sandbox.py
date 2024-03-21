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
    pygame.display.set_caption("Micromouse Simulator")    # タイトルを作成

    maze = Maze(0)
    mouse = Micromouse(maze)
    mouse.ground_noise_on()
    mouse.straight_demo(screen, flag = True)

if __name__ == "__main__":
    main() 
