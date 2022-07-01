Team control of two robots
Mahdi Abbasalipour 9823061
MohammadHossein Fattahi 9723061
In the relevant code, the movement of the robots is controlled by implementing the PID class.This is impelimented in PID.py .
Both angular and linear velocities are controlled by controllers.
The P controller has low speed and accuracy because its speed decreases as the robot approaches the desired state.
The integrator is used to improve the accuracy and the derivative is used to improve the speed of reaching the desired state.Due to the derivative property in increasing noise, a filter has been used next to it.
All controllers are discrete by the backward discretization method.In this way, the iddar controller may become unstable, but instability does not become stability.
The strategy of the attacking robot is that it is constantly chasing the ball and the defending robot goes to the goal after the start of the game and after settling there, follows the angle of the ball.


