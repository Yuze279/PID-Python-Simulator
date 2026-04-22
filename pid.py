import numpy as np 
import matplotlib 
import turtle 
import time 

#GLOBAL PARAMS 
TIMER = 0 
SETPOINT = 1000

# -----------

class Simulation(object): 
    def __init__(self):
        self.screen = turtle.Screen()
        self.screen.setup(1280, 900)
        self.marker = turtle.Turtle() 

    #     self.marker.penup()
    #     self.market.left(90)
    #     self.marker.goto(15,SETPOINT)
    #     self.marker.color('red')
    #     self.sim = True
    #     self.timer = 0

    # def cycle(self): 
    #     while(self.sim): 
    #         if self
    # #1 degree of freedom problem 2 PID problem


def main():
    while(TIMER < 5):
        sim = Simulation()
        time.sleep(1)
        timer +=1
    # sim.cycle()

main()