
from time import sleep
from gpiozero import Servo

#back bomb gpio 1, first 
#back bomb gpio 2, second

#backServo.min() , open
#frontServo.max(), open

class ServoControl():
    
    def __init__(self,my_GPIO1,my_GPIO2,min_pw,max_pw):
        
        self.my_GPIO1 = my_GPIO1
        self.my_GPIO2 = my_GPIO2
        self.min_pw = min_pw
        self.max_pw = max_pw

    def DropBackBomb(self):
        
        backServo = Servo(self.my_GPIO1, min_pulse_width = self.min_pw, max_pulse_width = self.max_pw)
        
        backServo.min()
        sleep(1)
        backServo.max()
        sleep(1)
        print('Arka bomba atılıyor')
        
        backServo.close()
        
    def DropFrontBomb(self):
        
        backServo = Servo(self.my_GPIO1, min_pulse_width = self.min_pw, max_pulse_width = self.max_pw)
        frontServo = Servo(self.my_GPIO2, min_pulse_width = self.min_pw, max_pulse_width = self.max_pw)
        
        backServo.min()  #open
        frontServo.min() #hold
        sleep(1)
        backServo.min()  #hold
        frontServo.max() #open
        sleep(1)
        backServo.min()  #hold
        frontServo.min() #close
        sleep(1)
        backServo.max()  #close
        frontServo.min() #hold
        sleep(1)
        
        print('Ön bomba atılıyor')
        
        backServo.close()
        frontServo.close()
    
    #Bu noktadan sonraki metotlar test amaçlıdır. Uçuşta kullanılmayacaktır.    
    def OpenFrontServo(self):
    
        backServo = Servo(self.my_GPIO1, min_pulse_width = self.min_pw, max_pulse_width = self.max_pw)
        frontServo = Servo(self.my_GPIO2, min_pulse_width = self.min_pw, max_pulse_width = self.max_pw)
        
        backServo.min()
        frontServo.min()
        sleep(1)
        backServo.min()
        frontServo.max()
        sleep(1)
        
        backServo.close()
        frontServo.close()
        
    def CloseFrontServo(self):
            
        backServo = Servo(self.my_GPIO1, min_pulse_width = self.min_pw, max_pulse_width = self.max_pw)
        frontServo = Servo(self.my_GPIO2, min_pulse_width = self.min_pw, max_pulse_width = self.max_pw)
        
        backServo.min()
        frontServo.min()
        sleep(1)
        backServo.max()
        frontServo.min()
        sleep(1)
        
        backServo.close()
        frontServo.close()
        
    def OpenBackServo(self):
    
        backServo = Servo(self.my_GPIO1, min_pulse_width = self.min_pw, max_pulse_width = self.max_pw)
        
        backServo.min()
        sleep(1)

        backServo.close()
        
    def CloseBackServo(self):
    
        backServo = Servo(self.my_GPIO1, min_pulse_width = self.min_pw, max_pulse_width = self.max_pw)
        
        backServo.max()
        sleep(1)

        backServo.close()
        
        
my_GPIO1 = 23 # 8 
my_GPIO2 = 24 # 9
correction = 0.50
max_pw = (2 + correction)/1000
min_pw = (1 - correction)/1000       
        
a1 = ServoControl(my_GPIO1,my_GPIO2,min_pw,max_pw)
a1.DropBackBomb()
