# rcj_soccer_player controller - ROBOT B1

# Feel free to import built-in libraries
from ast import And
import math  # noqa: F401

# You can also import scripts that you put into the folder with controller
import utils
from rcj_soccer_robot import RCJSoccerRobot, TIME_STEP
from cmath import atan, e
import math
from os import times_result  # noqa: F401

# You can also import scripts that you put into the folder with controller
import utils
from rcj_soccer_robot import RCJSoccerRobot, TIME_STEP
from time import time, sleep
from PID import PID
from math import sqrt, atan2


class MyRobot2(RCJSoccerRobot):
    def run(self):
        T = 0.0010
        L = 0.085
        R = 0.02
        xd = 0.7
        yd = 0.0
        vl = 0.0
        vr = 0.0
        w = [0, 0]
        u = [0, 0]
        e_r = [0, 0]
        delta=0
        e_p = [0, 0]
        robot_pos = self.get_gps_coordinates()
        robot_com = self.get_compass_heading()
        C_r = PID(kd=0.4, kp=0.3, ki=0.3, T=T)
        C_p = PID(kd=1.2, kp=4, ki=0.1, T=T)
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
                if abs(robot_pos[1]-xd)<0.1 and abs(robot_pos[0]-yd)<0.1 and ball_data['strength']>90:
                    u=[5,5]


                elif abs(robot_pos[1]-xd)<0.1 and abs(robot_pos[0]-yd)<0.1 :
                    e_p[1]=-ball_data["direction"][1]
                    w=C_p.run(e_p)
                    u=[u[0]/4,u[1]/4]
               

                   

                else :
                    e_r[1] = sqrt(
                        (robot_pos[0] - yd) ** 2 + (robot_pos[1] - xd) ** 2
                    )

                    
                    if (yd<robot_pos[0])and xd>robot_pos[1]:
                        delta=robot_com - atan2(
                        -yd + robot_pos[0], -xd + robot_pos[1]
                         )
                    else:
                        delta=robot_com - atan2(
                        yd - robot_pos[0], -xd + robot_pos[1]
                        )
                    e_p[1] =delta
                    if xd>robot_pos[1]:
                        e_p[1] = robot_com +3.1415- atan2(
                        yd - robot_pos[0], -xd + robot_pos[1]
                        )

                    if abs(e_p[1]) > 0.2 and (abs(e_r[1]) > 0.1) :
                        u[0]=u[0]/2
                        u[1]=u[1]/2
                        w = C_p.run(e_p)                      
                    else :
                        u = C_r.run(e_r)

               
                
                    
                vr_ = (2 * -u[1] + L * w[1]) / (2 * R)
                vl_ = (2 * -u[1] - L * w[1]) / (2 * R)

                # ****************dea zone:
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
     




