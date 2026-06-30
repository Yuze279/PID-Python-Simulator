from matplotlib import pyplot as plt
import numpy as np 
import matplotlib 
import turtle 
import time 

#GLOBAL PARAMS 
TIMER = 0 
SETPOINT = 10 #final goal 
SIM_TIME = 1000    # in sec

TIME_STEP = 0.005

#_______
INITIAL_X = 0 
INITIAL_Y = -100
MASS = 1 #kg
MAX_THRUST = 15 #Newtons 
g = -9.81 # Gravitational constant

V_i = 0 #initial velocity
Y_i = 0 #initial height

#---PID GAINS---
KP = 1.0
KI = 1.0
KD = 1.0


# -----------

class Simulation(object): 
    def __init__(self):

        self.Insight = Rocket()
        self.pid = PID(KP, KI, KD, SETPOINT)

        self.screen = turtle.Screen()
        self.screen.setup(1280, 900)
        self.marker = turtle.Turtle() 

        #Goal we are getting to (setpoint/ marker)
        self.marker.penup()
        self.marker.left(180)
        self.marker.goto(15,SETPOINT)
        self.marker.color('red')
        self.sim = True
        self.timer = 0
        self.poses = np.array([])
        self.times = np.array([])


    #do our simulation cycles?
    def cycle(self): 
        while(self.sim): 
        
            #get a thrust output from our PID
            thrust = self.pid.computer(self.Insight.get_y())
            print(thrust)
            
            #10 #newtons
            self.Insight.set_ddy(thrust)
            self.Insight.set_dy()
            self.Insight.set_y()
            time.sleep(TIME_STEP)
            self.timer +=1 

            if self.timer > SIM_TIME: 
                print("SIM ENDED")
                self.sim = False
            
            elif self.Insight.get_y() > 800: 
                print("OUT OF BOUNDS")
                self.sim = False
            
            elif self.Insight.get_y() < -800: 
                print("OUT OF BOUNDS")
                self.sim = False
            
            self.poses = np.append(self.poses, self.Insight.get_y())
            self.times = np.append(self.times, self.timer)

        graph(self.times, self.poses)
            
    #1 degree of freedom problem 2 sep PID problem


#matplotlib
def graph(x, y): 
    plt.plot(x, y)
    plt.show()



#thrust verc control rocket, Integral builds over time, how far away for how long
#-->Gimple for 3 sec only 3 sec for integral error, PD controler 
# misalignment, (Slowly correct for upright minimize translational motion)
class Rocket(object):
    def __init__(self):
        global Rocket
        self.Rocket = turtle.Turtle()
        self.Rocket.shape('square')
        self.Rocket.color('black')
        self.Rocket.penup()
        self.Rocket.goto(INITIAL_X, INITIAL_Y)
        self.Rocket.speed(0)

        #physics
        self.ddy = 0 # v acceleration
        self.dy = V_i # ver velocity
        self.y = INITIAL_Y
        #Y_i #


    def set_ddy(self, thrust): # v acceleration
        self.ddy = g + thrust/MASS # thrust impact system ==> output of PID controller

    def get_ddy(self):
        return self.ddy 

    def set_dy(self):
        self.dy += self.ddy #2nd derivate ddy has it's own Intergral this is  Velocity
   
    def get_dy(self):
        return self.dy


    def set_y(self):
        # self.y += self.dy
        self.Rocket.sety(self.y + self.dy)
        # return self.y

    def get_y(self):
        self.y = self.Rocket.ycor()
        return self.y


class PID(object): 
    def __init__(self, KP, KI, KD, target): 
        self.kp = KP
        self.ki = KI 
        self.kd = KD # quickly adjust for subpoint swing or overshoot subpoint
        self.setpoint = target 
        self.error = 0
        self.integral_error = 0
        
        self.error_last = 0
        self.derivative_error = 0 
        self.output = 0
    #PID output computation
    def computer(self, pos):
        self.error = self.setpoint - pos 
        self.integral_error += self.error * TIME_STEP #sum of this over time ==> error w/respect to time
        self.derivative_error = (self.error - self.error_last) / TIME_STEP

        self.error_last = self.error


        #PID equation

        self.output = self.kp * self.error + self.ki * self.integral_error + self.kd * self.derivative_error


        if self.output >= MAX_THRUST: 
            self.output = MAX_THRUST
        
        elif self.output <= 0: 
            self.output = 0
        return self.output 



def main():
    # while(TIMER < 5):
    sim = Simulation()
    #     time.sleep(1)
    #     timer +=1
    sim.cycle()

main()