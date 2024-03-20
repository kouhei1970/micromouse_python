from  micromouse import Micromouse
from maze import Maze
import matplotlib.pyplot as plt
import numpy as np
#from random import gauss
#from pygame.locals import *
#import pygame
#import sys

def main():
    maze = Maze(2)
    maze.draw_map()
    x = np.linspace(0,2*np.pi)
    y = np.sin(x)
    plt.plot(x ,y)
    plt.show()
    
    

if __name__ == "__main__":
    main()