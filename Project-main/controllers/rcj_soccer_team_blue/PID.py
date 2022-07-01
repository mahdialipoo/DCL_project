
class PID():
    def __init__(self,kd,kp,ki,T,N=100.0) :
        self.kd=kd
        self.kp=kp
        self.ki=ki
        self.T=T
        self.I=0
        self.y=[0,0]
        self.D=0.0
        self.N=N
    def run(self,x):

        self.y[0]=self.y[1]
        self.y[1]=self.kp*x[1]+self.I+self.kd*self.D
        self.I=self.ki*self.T*x[1]+self.I
        self.D=(self.D+x[1]-x[0])/(self.N*self.T+1)
        x[0]=x[1]
        return self.y
        
