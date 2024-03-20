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
    
def pause_plot():
    fig, ax = plt.subplots(1, 1)
    x = np.arange(-np.pi, np.pi, 0.1)
    y = np.sin(x)
    # 初期化的に一度plotしなければならない
    # そのときplotしたオブジェクトを受け取る受け取る必要がある．
    # listが返ってくるので，注意
    lines, = ax.plot(x, y)

    # ここから無限にplotする
    while True:
        # plotデータの更新
        #x += 0.1
        y = np.sin(x)

        # 描画データを更新するときにplot関数を使うと
        # lineオブジェクトが都度増えてしまうので，注意．
        #
        # 一番楽なのは上記で受け取ったlinesに対して
        # set_data()メソッドで描画データを更新する方法．
        lines.set_data(x, y)

        # set_data()を使うと軸とかは自動設定されないっぽいので，
        # 今回の例だとあっという間にsinカーブが描画範囲からいなくなる．
        # そのためx軸の範囲は適宜修正してやる必要がある．
        ax.set_xlim((x.min(), x.max()))

        # 一番のポイント
        # - plt.show() ブロッキングされてリアルタイムに描写できない
        # - plt.ion() + plt.draw() グラフウインドウが固まってプログラムが止まるから使えない
        # ----> plt.pause(interval) これを使う!!! 引数はsleep時間
        plt.pause(.01)

stop = False

def on_close(event):
    global stop
    stop = True
    pass  # ここで終了イベントを受け取って渡す？


def main2():
    plt.ion()
    fig = plt.figure()
    fig.canvas.mpl_connect('close_event', on_close)
    ax = fig.add_subplot()
    bg = fig.canvas.copy_from_bbox(ax.bbox)
    line, = ax.plot(np.linspace(0, 99, 100), np.zeros(100))
    ax.set_ylim(0.0, 1.0)
    for i in range(1000):
        if stop == True:
            break
        line.set_ydata(np.random.rand(100))
        fig.canvas.restore_region(bg)
        fig.canvas.blit(ax.bbox)
        fig.canvas.flush_events()
        

if __name__ == "__main__":
    main2() 
