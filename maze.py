import numpy as np
import matplotlib.pyplot as plt
from random import gauss
from pygame.locals import *
import pygame
import sys

def normal_vector(v1,v2):
        #print(v1)
        #print(v2)
        normal_x = v1[1] - v2[1]
        normal_y = v2[0] - v1[0]
        norm = np.sqrt(normal_x**2 + normal_y**2)
        normal_x = normal_x / norm
        normal_y = normal_y / norm
        return normal_x, normal_y

class Wall():
    def __init__(self, x ,y, angle):
        #オリジナルは横置き
        #最大長の180mmを1として正規化（ハーフ，クラシックに対応のため）
        base_size =180.0
        #vertex
        vertex = np.array([[84.0/base_size, 84.0/base_size,-84.0/base_size,-84.0/base_size],
                             [6.0/base_size, -6.0/base_size, -6.0/base_size,  6.0/base_size]])
        vertex = vertex.dot(base_size)
        self.pos = np.array([[x], [y]])
        self.angle = angle
        rmat = np.array([[np.cos(angle), -np.sin(angle)],[np.sin(angle), np.cos(angle)]])
        vertex = rmat.dot(vertex)
        vertex[0,:] = vertex[0,:] + x
        vertex[1,:] = vertex[1,:] + y
        self.vertex = vertex
        #print(self.vertex)
        #Normal vector
        n0x, n0y = normal_vector(self.vertex[:,0], self.vertex[:,1])
        n1x, n1y = normal_vector(self.vertex[:,1], self.vertex[:,2])
        n2x, n2y = normal_vector(self.vertex[:,2], self.vertex[:,3])
        n3x, n3y = normal_vector(self.vertex[:,3], self.vertex[:,0])
        self.normal = np.array([[n0x, n1x, n2x, n3x],
                                [n0y, n1y, n2y, n3y]])

class Colmun():
    def __init__(self,x ,y, angle):
        #壁と同様に180mmを1として正規化
        base_size =180.0
        #vertex
        vertex = np.array([[6.0/base_size, 6.0/base_size,-6.0/base_size,-6.0/base_size],
                       [6.0/base_size,-6.0/base_size,-6.0/base_size, 6.0/base_size]])
        vertex = vertex.dot(base_size)
        self.pos = np.array([[x], [y]])
        self.angle = angle
        rmat = np.array([[np.cos(angle), -np.sin(angle)],[np.sin(angle), np.cos(angle)]])
        vertex = rmat.dot(vertex)
        vertex[0,:] = vertex[0,:] + x
        vertex[1,:] = vertex[1,:] + y
        self.vertex = vertex
        #print(self.vertex)
        #Normal vector
        n0x, n0y = normal_vector(self.vertex[:,0], self.vertex[:,1])
        n1x, n1y = normal_vector(self.vertex[:,1], self.vertex[:,2])
        n2x, n2y = normal_vector(self.vertex[:,2], self.vertex[:,3])
        n3x, n3y = normal_vector(self.vertex[:,3], self.vertex[:,0])
        self.normal = np.array([[n0x, n1x, n2x, n3x],
                                [n0y, n1y, n2y, n3y]])

class Maze():
    #self.map=Mapdata()
    def __init__(self, n):
        self.read_map(n)
        self.map = []
        self.make_map()

    def read_map(self, x):
        if x ==0:
            #2017Clacic
            N = 16
            M = 16 
            #2017 Clasic mouse expart final maze
            self.smap=[
                "+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+",#    0
                "|   |                                                           |",#15  1
                "+   +   +   +---+---+---+---+---+---+---+---+---+---+   +   +---+",#    2
                "|       |   |               |               |       |   |       |",#14  3
                "+   +---+   +   +---+---+   +   +---+---+   +   +   +---+---+   +",#    4
                "|           |   |       |       |       |       |           |   |",#13  5
                "+   +---+   +   +   +   +---+---+   +   +---+---+---+---+   +   +",#    6
                "|           |       |               |               |       |   |",#12  7
                "+   +---+---+---+---+---+---+---+---+---+---+---+   +   +---+   +",#    8
                "|   |       |       |           |       |   |       |       |   |",#11  9
                "+   +   +   +   +   +   +   +   +   +   +   +   +---+---+   +   +",#   10
                "|   |   |       |       |   |       |           |           |   |",#10 11
                "+   +   +---+---+---+---+---+---+---+---+---+---+---+   +---+   +",#   12
                "|   |       |           |               |       |           |   |",# 9 13
                "+   +---+   +   +   +---+   +   +---+   +   +   +---+   +---+   +",#   14
                "|   |       |   |       |   |       |       |               |   |",# 8 15
                "+   +   +---+   +---+   +   +       +---+---+---+---+   +---+   +",#   16
                "|   |   |       |       |   |       |   |                   |   |",# 7 17
                "+   +   +   +---+   +---+   +---+---+   +   +   +---+---+---+   +",#   18
                "|   |       |           |                   |               |   |",# 6 19
                "+   +---+---+   +---+---+---+---+---+---+---+---+---+---+   +   +",#   20
                "|   |           |       |   |       |   |           |       |   |",# 5 21
                "+   +   +---+---+   +   +   +   +   +   +   +   +---+   +---+   +",#   22
                "|   |       |       |           |           |   |   |       |   |",# 4 23
                "+   +---+   +   +---+---+   +---+---+   +---+   +   +---+   +   +",#   24
                "|   |       |       |   |   |       |   |       |           |   |",# 3 25
                "+   +   +---+---+   +   +---+   +   +---+   +---+   +---+---+   +",#   26
                "|   |   |           |           |           |   |   |           |",# 2 27
                "+   +   +   +---+---+   +---+---+---+---+---+   +   +   +   +   +",#   28
                "|       |   |   |   |                               |   |   |   |",# 1 29
                "+   +---+   +   +   +---+---+---+---+---+---+---+---+---+---+   +",#   30
                "|   |                                                           |",# 0 31
                "+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+"#    32
                #  0   1   2   3   4   5   6   7   8   9  10  11  12  13  14  15
            ]
        elif x == 1:
            N = 3
            M = 3 
            self.smap=[
                "+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+",#    0
                "|   |                                                           |",#15  1
                "+   +   +   +---+---+---+---+---+---+---+---+---+---+   +   +---+",#    2
                "|       |   |               |               |       |   |       |",#14  3
                "+   +---+   +   +---+---+   +   +---+---+   +   +   +---+---+   +",#    4
                "|           |   |       |       |       |       |           |   |",#13  5
                "+   +---+   +   +   +   +---+---+   +   +---+---+---+---+   +   +",#    6
                "|           |       |               |               |       |   |",#12  7
                "+   +---+---+---+---+---+---+---+---+---+---+---+   +   +---+   +",#    8
                "|   |       |       |           |       |   |       |       |   |",#11  9
                "+   +   +   +   +   +   +   +   +   +   +   +   +---+---+   +   +",#   10
                "|   |   |       |       |   |       |           |           |   |",#10 11
                "+   +   +---+---+---+---+---+---+---+---+---+---+---+   +---+   +",#   12
                "|   |       |           |               |       |           |   |",# 9 13
                "+   +---+   +   +   +---+   +   +---+   +   +   +---+   +---+   +",#   14
                "|   |       |   |       |   |       |       |               |   |",# 8 15
                "+   +   +---+   +---+   +   +       +---+---+---+---+   +---+   +",#   16
                "|   |   |       |       |   |       |   |                   |   |",# 7 17
                "+   +   +   +---+   +---+   +---+---+   +   +   +---+---+---+   +",#   18
                "|   |       |           |                   |               |   |",# 6 19
                "+   +---+---+   +---+---+---+---+---+---+---+---+---+---+   +   +",#   20
                "|   |           |       |   |       |   |           |       |   |",# 5 21
                "+   +   +---+---+   +   +   +   +   +   +   +   +---+   +---+   +",#   22
                "|   |       |       |           |           |   |   |       |   |",# 4 23
                "+   +---+   +   +---+---+   +---+---+   +---+   +   +---+   +   +",#   24
                "|   |       |       |   |   |       |   |       |           |   |",# 3 25
                "+---+---+---+---+   +   +---+   +   +---+   +---+   +---+---+   +",#   26
                "|       |   |       |           |           |   |   |           |",# 2 27
                "+   +---+   +---+---+   +---+---+---+---+---+   +   +   +   +   +",#   28
                "|       |   |   |   |                               |   |   |   |",# 1 29
                "+   +   +   +   +   +---+---+---+---+---+---+---+---+---+---+   +",#   30
                "|   |       |                                                   |",# 0 31
                "+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+"#    32
                #  0   1   2   3   4   5   6   7   8   9  10  11  12  13  14  15
            ]
        elif x==2:
            #2015Clacic
            M = 16
            N = 16
            self.smap=[
                "+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+",#    0
                "|                                                               |",#15  1
                "+   +---+---+---+---+---+---+---+---+---+---+---+---+---+---+   +",#    2
                "|   |                                                       |   |",#14  3
                "+   +   +---+---+---+---+---+---+---+---+---+---+---+   +---+   +",#    4
                "|   |   |       |                                   |       |   |",#13  5
                "+   +   +   +   +   +   +---+---+---+---+   +---+   +---+   +   +",#    6
                "|   |   |   |   |   |                       |   |       |       |",#12  7
                "+   +   +   +   +   +---+   +---+---+   +---+   +---+   +---+   +",#    8
                "|   |   |   |   |       |       |       |           |       |   |",#11  9
                "+   +   +   +   +   +---+---+   +   +---+   +   +---+   +---+   +",#   10
                "|   |   |   |   |           |       |       |           |   |   |",#10 11
                "+   +   +   +   +   +---+---+   +   +---+   +---+   +---+   +---+",#   12
                "|   |   |   |           |       |       |       |   |           |",# 9 13
                "+   +   +   +   +---+---+   +---+---+   +---+   +---+---+   +---+",#   14
                "|   |   |   |       |       |       |       |       |           |",# 8 15
                "+   +   +   +   +---+   +---+       +---+   +---+   +   +---+   +",#   16
                "|   |   |       |       |               |       |       |       |",# 7 17
                "+   +   +   +---+   +---+   +---+---+   +---+   +---+---+   +---+",#   18
                "|   |       |       |   |       |       |   |       |           |",# 6 19
                "+   +   +---+   +---+   +---+   +   +---+   +---+   +---+   +---+",#   20
                "|   |   |       |           |       |           |       |       |",# 5 21
                "+   +---+   +---+   +   +---+   +   +---+   +   +---+   +---+   +",#   22
                "|   |       |       |           |           |       |       |   |",# 4 23
                "+   +---+   +   +---+---+   +---+---+   +---+---+   +   +---+   +",#   24
                "|   |           |   |           |           |   |       |       |",# 3 25
                "+   +---+   +   +   +   +---+   +   +---+   +   +   +---+   +---+",#   26
                "|       |   |           |   |       |   |           |           |",# 2 27
                "+---+   +   +---+   +---+   +---+---+   +---+   +---+---+   +---+",#   28
                "|       |           |           |           |   |       |       |",# 1 29
                "+   +   +---+---+---+   +---+   +   +---+   +---+   +   +---+   +",#   30
                "|   |                   |               |           |           |",# 0 31
                "+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+"#    32
                #  0   1   2   3   4   5   6   7   8   9  10  11  12  13  14  15
            ]

    def make_map(self):
        col_width = 12
        wall_width = 12
        wall_length = 180-12
        counter = 0

        y = 0
        for i in range(32,-1,-1):
            x = 0
            for j in range(0,65,2):
                xc = x*90.0
                yc = y*90.0
                if self.smap[i][j]=='+':
                    col = Colmun(xc, yc, 0.0)
                    self.map.append(col)
                elif self.smap[i][j]=='-':
                    wall = Wall(xc, yc, 0.0)
                    self.map.append(wall)
                elif self.smap[i][j]=='|':
                    wall = Wall(xc, yc, np.pi/2)
                    self.map.append(wall)
                else:
                    self.map.append([])
                x += 1
            y += 1


    def printmap(self):
        for i in range(33):
            print(self.smap[i])

    def raycast(self, pos, rayvect, index):
        if self.map[index]==[]:
            return 0, 0, False
        
        n = self.map[index].normal
        vertex = self.map[index].vertex
        num = (n.T).dot(pos-vertex)
        num = np.diag(num)
        den = (n.T).dot(rayvect)
        den = den.T
        t0 = - num/(den+1e-16)
        t = t0
        p = pos + rayvect.dot(t)
        index =[]
        for i in range(4):
            if i==3:
                if (p[:,3]-vertex[:,3]).dot(p[:,3]-vertex[:,0])>0:
                    index.append(False)
                else:
                    index.append(True)
            else:
                if (p[:,i]-vertex[:,i]).dot(p[:,i]-vertex[:,i+1])>0:
                    index.append(False)
                else:
                    index.append(True)
        t = t[0,index]
        p = p[:,index]
        index = t>0
        t = t[index]
        p = p[:, index]
        if t.size ==0:
            return 0,0,False
        
        index = np.argmin(t)
        t = t[index]
        p = p[:,index]
        flag = True
        return t, p, flag

    def draw_map(self):
        pygame.init()    # Pygameを初期化
        clock = pygame.time.Clock()
        screen = pygame.display.set_mode((640, 640))    # 画面を作成
        pygame.display.set_caption("Micromouse Simulation")    # タイトルを作成
        FPS = 30
        w_width = 640
        w_height =640
        maze_width = (180*16)/5
        maze_height = (180*16)/5
        running = True
        time = 0.0
        render_step = 1/FPS
        white= (255,255,255)
        black = (0,0,0)
        #Rendering
        mx=0
        my=0
        while True:
            screen.fill((0,0,0)) # 背景を黒で塗りつぶす
            for i in range(33*33):
                map_obj = self.map[i]
                if map_obj !=[]:
                    vertex = map_obj.vertex
                    vertex = vertex/5
                    vertex[0] = vertex[0] + (w_width - maze_width)/2
                    vertex[1] = 640 - vertex[1] - (w_height - maze_height)/2
                    vertex = vertex.T
                    vertex = vertex.tolist()
                    #print(vertex)
                    pygame.draw.polygon(screen, (200,0,0),vertex)
                    #screen.fill((200,0,0), (x, y, width, height))

            #Ray Draw
            for angle in range(0,360,5):
                pos = np.array([[90+180*mx],[90+180*my]])
                rayvect = np.array([[np.cos(angle*np.pi/180)],[np.sin(angle*np.pi/180)]])
                rayvect = rayvect/np.linalg.norm(rayvect)
                min_range = 10000.0
                min_p =[]
                for index in range(33*33):
                    t, p, flag = self.raycast(pos, rayvect, index)
                    if flag:
                        if min_range>t:
                            min_range = t
                            min_p = p
                pos = pos/5
                pos[0] = pos[0] + (w_width - maze_width)/2
                pos[1] = 640 - pos[1] - (w_height - maze_height)/2
                min_p = min_p/5
                min_p[0] = min_p[0] + (w_width - maze_width)/2
                min_p[1] = 640 - min_p[1] - (w_height - maze_height)/2
                pygame.draw.line(screen, (255, 255, 0), pos[:,0], min_p, 2)
            pygame.display.update() #描画処理を実行
            #clock.tick(FPS)

            for event in pygame.event.get():
                if event.type == QUIT:  # 終了イベント
                    pygame.quit()  #pygameのウィンドウを閉じる
                    sys.exit() #システム終了
            mx = int(np.random.random(1)*16)
            my = int(np.random.random(1)*16)
            '''
            my += 1
            if my==16:
                my=0
                mx+=1
                if mx==16:
                    mx = 0
            '''
        #End of Loop
        pygame.quit()  #pygameのウィンドウを閉じる
        #print(ref_psi*180/np.pi)

def main():
    maze = Maze(2)
    maze.draw_map()

if __name__ == "__main__":
    main()