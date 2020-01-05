from math import cos, sin, pi

class unitSwitch:
    #PI = 3.1415926
    def degreeToSec(self,deg=0.0,mins=0.0,sec=0.0):
        return deg*3600 + mins*60 + sec
    def degreeToRad(self,deg=0.0,mins=0.0,sec=0.0):
        return pi/180 * (deg+mins/60+sec/3600)
    def secToRad(self,sec=0.0):
        return sec*pi/648000
    def degreeToDeg(self,deg=0.0,mins=0.0,sec=0.0):
        return deg + mins/60 + sec/3600

class LBtoxy:
    
   
    def __init__(self,L=0.0,B=0.0,distinguish=6):
        self.RHO = 206265
        self.switch = unitSwitch()        
        self.myL = L
        self.myB = B
        self.distinguish = distinguish
        self.myx = 0.0
        self.myy = 0.0       

    def setOrder(self):
        if self.distinguish == 6:
            self.myOrder = int(self.myL/6) + 1
        else:
            self.myOrder = int(self.myL/3 + 0.5)
           
    def setL0(self):
        if self.distinguish == 6:
            self.myL0 = 6*self.myOrder - 3
        else:
            self.myL0 = 3*self.myOrder

    def setl(self):
        self.myl = self.switch.degreeToSec(self.myL-self.myL0)/self.RHO
    def setN(self):
        self.myN = 6399698.902 - (21562.267 - (108.973 - 0.612 * self.myCosBSquare) * self.myCosBSquare)* self.myCosBSquare    
    def seta0(self):
        self.mya0 = 32140.404 - (135.3302 - (0.7092 - 0.0040 * self.myCosBSquare)* self.myCosBSquare)* self.myCosBSquare

    def seta4(self):
        self.mya4 = (0.25 + 0.00252 * self.myCosBSquare) * self.myCosBSquare - 0.04166;

    def seta6(self):
        self.mya6  = (0.166 * self.myCosBSquare - 0.084) * self.myCosBSquare;

    def seta3(self):
        self.mya3 = (0.3333333 + 0.001123 * self.myCosBSquare)* self.myCosBSquare - 0.1666667;

    def seta5(self):
        self.mya5 = 0.0083 - (0.1667 -(0.1968 +0.0040 * self.myCosBSquare)* self.myCosBSquare) * self.myCosBSquare;

    def setCosBandSinB(self):
        self.myCosB = cos(self.switch.degreeToRad(self.myB));
        self.mySinB = sin(self.switch.degreeToRad(self.myB));

        self.myCosBSquare = self.myCosB * self.myCosB;
        self.mySinBSquare = self.mySinB * self.mySinB;

    def setGaussCoordinate(self):
        self.myx = 6367558.4969 * self.switch.degreeToSec(self.myB) / self.RHO - (self.mya0 - (0.5 + (self.mya4 + self.mya6 * self.myl**2) * self.myl**2)
        * self.myl**2 * self.myN)* self.mySinB * self.myCosB;
        self.myy = (1 + (self.mya3 + self.mya5 * self.myl**2)* self.myl**2)* self.myl * self.myN * self.myCosB;
    def calculateAll(self):
        self.setOrder();
        self.setL0();
        self.setCosBandSinB();
        self.setl();
        self.setN();
        self.seta0();
        self.seta4();
        self.seta6();
        self.seta3();
        self.seta5();
        self.setGaussCoordinate();
    def getOrder(self):
        return self.myOrder
    def getl(self):
        return self.myl
    def getx(self):
        return self.myx
    def gety(self):
        return self.myy
    
if __name__ == '__main__':
    t=LBtoxy(113.6666,23.2222)
    t.calculateAll()
    print t.myx, t.myy