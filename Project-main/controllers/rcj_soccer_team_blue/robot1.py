# rcj_soccer_player controller - ROBOT B1

# Feel free to import built-in libraries
import math  # noqa: F401

# You can also import scripts that you put into the folder with controller
import utils
from rcj_soccer_robot import RCJSoccerRobot, TIME_STEP
from cmath import atan
import math
from os import times_result  # noqa: F401

# You can also import scripts that you put into the folder with controller
import utils
from rcj_soccer_robot import RCJSoccerRobot, TIME_STEP
from time import time, sleep
from PID import PID
from math import sqrt, atan2


class MyRobot1(RCJSoccerRobot):
    def run(self):
        T= 0.0010
        L = 0.085
        R = 0.02
        xd = -0.3
        yd = 0.4
        vl = 0.0
        vr = 0.0
        w = [0, 0]
        u = [0, 0]
        e_r = [0, 0]
        e_p = [0, 0]
        robot_pos = self.get_gps_coordinates()
        robot_com = self.get_compass_heading()
        C_r = PID(kd=0.05, kp=0.2, ki=0.3, T=T)
        C_p = PID(kd=1.2, kp=2, ki=0.09, T=T)
        while self.robot.step(TIME_STEP) != -1:
            start = time()
            if self.is_new_data():
                data = self.get_new_data()  # noqa: F841

                while self.is_new_team_data():
                    team_data = self.get_new_team_data()  # noqa: F841
                    # Do something with team data

                if self.is_new_ball_data():
                    ball_data = self.get_new_ball_data()
                else:
                    # If the robot does not see the ball, stop motors
                    self.left_motor.setVelocity(0)
                    self.right_motor.setVelocity(0)
                    continue

                # Get data from compass
                heading = self.get_compass_heading()  # noqa: F841

                # Get GPS coordinates of the robot
                robot_pos = self.get_gps_coordinates()  # noqa: F841

                # Get data from sonars
                sonar_values = self.get_sonar_values()  # noqa: F841

                # Compute the speed for motors
                direction = utils.get_direction(ball_data["direction"])

                # Write scripts of the robot here
                 # ******************************************************************mycode
                robot_pos = self.get_gps_coordinates()
                robot_com = self.get_compass_heading()
                e_r[1] = 1-ball_data["strength"]/100
                e_p[1]=-ball_data["direction"][1]
                if  abs(e_r[1]) <0.2 :
                    w=[0,0]
                    u=[5,5]
                elif (abs(e_p[1]) > 0.1) and (abs(e_r[1]) > 0.3 and ball_data["direction"][0]>-3.14):
                    u[0]=u[0]/2
                    u[1]=u[1]/2
                    w = C_p.run(e_p)  
                elif (abs(e_p[1]) > 0.1) and (abs(e_r[1]) > 0.3 and ball_data["direction"][0]<-3.14):
                    u[0]=u[0]/2
                    u[1]=u[1]/2
                    e_p[1]=2
                    w = C_p.run(e_p)   
                elif (abs(e_p[1]) < 0.01) and (abs(e_r[1]) > 0.3):
                    u = C_r.run(e_r)
               
                
                    
                vr_ = (2 * u[1] + L * w[1]) / (2 * R)
                vl_ = (2 * u[1] - L * w[1]) / (2 * R)

                # ****************dead zone:
                if abs(vl_) > 0.1:
                    vl = vl_
                else:
                    vl = vl * 0.8
                if abs(vr_) > 0.1:
                    vr = vr_
                else:
                    vr = vr * 0.8
                self.left_motor.setVelocity(vl)
                self.right_motor.setVelocity(vr)
                end = time()
                sleep(T - (end - start))
