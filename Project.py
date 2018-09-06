import RPi.GPIO as GPIO
import wiringpi
import PyLora
import time
from time import sleep 


GPIO.setmode(GPIO.BOARD)

PyLora.init()                      #initialize wireless connection
PyLora.set_frequency(868000000)
PyLora.enable_crc()


#lora connection 
#CS = 22
#RST = 11
#MISO = 21
#MOSI = 19
#SCK =23

RMotorDriverA=31        #Wheeling motors pins connection with motors drivers
RMotordriverB=33
RMotorDir=35
RMotorPwm=37

LMotorDriverA=7
LMotorDriverB=13
LMotorDir=15
LMotorPwm=16


GPIO.setup(RMotorDriverA,GPIO.OUT)
GPIO.setup(RMotorDriverB,GPIO.OUT)
GPIO.setup(RMotorDir,GPIO.OUT)
GPIO.setup(RMotorPwm,GPIO.OUT)

GPIO.setup(LMotorDriverA,GPIO.OUT)
GPIO.setup(LMotorDriverB,GPIO.OUT)
GPIO.setup(LMotorDir,GPIO.OUT)
GPIO.setup(LMotorPwm,GPIO.OUT)


pwmPin=18        #servo motor 
delay_period = 0.001
bound1=50                                            #You choose the boundries of the servo by changing in bound1 and bound2
bound2=120
pos=(bound1+bound2)/2                                 #the positon is first initialized as in the middle between bound1 and bound2
moving_const = 2
#pulseconst=0
#pulseconst2=60

wiringpi.wiringPiSetupGpio()
wiringpi.pinMode(18, wiringpi.GPIO.PWM_OUTPUT)

# set the PWM mode to milliseconds stype
wiringpi.pwmSetMode(wiringpi.GPIO.PWM_MODE_MS)

# divide down clock
wiringpi.pwmSetClock(192)
wiringpi.pwmSetRange(2000)




                    #Stepper motors 
DIR1=36
DIR2=38
DIR3=40 

clk1=12
clk2=29
clk3=32

CW=1
CCW=0
step_count=5             #Decrease to get more steps every signal user sent 
delay=.002               #determine the freq of the steps

GPIO.setup(DIR1, GPIO.OUT)
GPIO.setup(clk1, GPIO.OUT)
GPIO.output(DIR1,CW)

GPIO.setup(DIR2, GPIO.OUT)
GPIO.setup(clk2, GPIO.OUT)
GPIO.output(DIR2,CW)


GPIO.setup(DIR3, GPIO.OUT)
GPIO.setup(clk3, GPIO.OUT)
GPIO.output(DIR3,CW)






def forward(pwm):                                            #driving the wheels forward or backward or the braking system
    GPIO.output(RMotorDriverB,1)
    GPIO.output(LMotorDriverB,1)
    GPIO.output(RMotorDriverA,0)
    GPIO.output(LMotorDriverA,0)    
    GPIO.output(RMotorDir,0)
    GPIO.output(LMotorDir,0)
    RMotorPwm.ChangeDutyCycle(pwm)
    LMotorPwm.ChangeDutyCycle(pwm)
    #print ("forward")

def backward(pwm):
    GPIO.output(RMotorDriverB,0)
    GPIO.output(LMotorDriverB,0)
    GPIO.output(RMotorDriverA,1)
    GPIO.output(LMotorDriverA,1)    
    GPIO.output(RMotorDir,1)
    GPIO.output(LMotorDir,1)
    RMotorPwm.ChangeDutyCycle(pwm)
    LMotorPwm.ChangeDutyCycle(pwm)
    #print ("backward")



def brake():
    GPIO.output(RMotorDriverB,0)
    GPIO.output(LMotorDriverB,0)
    GPIO.output(RMotorDriverA,0)
    GPIO.output(LMotorDriverA,0)
    RMotorPwm.ChangeDutyCycle(0)
    LMotorPwm.ChangeDutyCycle(0)
    #print ("brake")
	

def servo_right():                                       #steering (using servo) functions for the right and left directions
    global pos
    #global pulseconst
    if pos<bound2:
                 wiringpi.pwmWrite(18, pos)
                 pos=pos+moving_const
                 #time.sleep(delay_period)
                 #print"servo_right"
                 #pulseconst=pulseconst+30
                 #if(pulseconst>30):
                     #pulseconst=0
                     #pulse=pulse+1
                 
    else:
        wiringpi.pwmWrite(18, 0)

def servo_left():
    global pos
    #global pulseconst2
    if pos>bound1:
                wiringpi.pwmWrite(18, pos)
                pos=pos-moving_const
                #time.sleep(delay_period)
                #print"servo_left"
                #pulseconst2=pulseconst2-30
                #if(pulseconst2<30):
                     #pulseconst2=60
                     #pulse=pulse-1
    else:
        wiringpi.pwmWrite(18, 0)    


def baseStepperCW():                                       #base stepper clockwise and counter clockwise functions
    GPIO.output(DIR1,CW)
    for i in range(step_count):
        GPIO.output(clk1,GPIO.HIGH)
        sleep(delay)
        GPIO.output(clk1,GPIO.LOW)
        sleep(delay)

def baseStepperCCW():
    GPIO.output(DIR1,CCW)
    for i in range(step_count):
        GPIO.output(clk1,GPIO.HIGH)
        sleep(delay)
        GPIO.output(clk1,GPIO.LOW)
        sleep(delay)

def midStepperCW():                                        #mid stepper clockwise and counter clockwise functions
    GPIO.output(DIR2,CW)
    for i in range(step_count):
        GPIO.output(clk2,GPIO.HIGH)
        sleep(delay)
        GPIO.output(clk2,GPIO.LOW)
        sleep(delay)

def midStepperCCW():
    GPIO.output(DIR2,CCW)
    for i in range(step_count):
        GPIO.output(clk2,GPIO.HIGH)
        sleep(delay)
        GPIO.output(clk2,GPIO.LOW)
        sleep(delay)

def Stepper3CW():                                            #First stepper clockwise and counter clockwise functions
    GPIO.output(DIR3,CW)
    for i in range(step_count):
        GPIO.output(clk3,GPIO.HIGH)
        sleep(delay)
        GPIO.output(clk3,GPIO.LOW)
        sleep(delay)

def Stepper3CCW():
    GPIO.output(DIR3,CCW)
    for i in range(step_count):
        GPIO.output(clk3,GPIO.HIGH)
        sleep(delay)
        GPIO.output(clk3,GPIO.LOW)
        sleep(delay)         

#Dartgun=28
#GPIO.setup(Dartgun,GPIO.OUT)

#def trigger_dartgun():
    #GPIO.output(Dartgun,GPIO.HIGH)
    #sleep(.1)
    #GPIO.output(Dartgun,GPIO.LOW)



OnFlag=0                          #The robot  is off  
while 1:
    try:
        PyLora.receive()   # put into receive mode
        while not PyLora.packet_available():
        # wait for a package
            time.sleep(0)
        data = PyLora.receive_packet()
        if (data=="on"):                 #Turning on the whole system
            OnFlag=1
        if (data=="off"):
            OnFlag=0
            GPIO.cleanup()
        if (OnFlag==1):                      
            if(data=="baseStepperCW"):                  #Stepper motors conrol
                baseStepperCW()
            if(data=="baseStepperCCW"):
                baseStepperCCW()
            if(data=="midStepperCW"):
                midStepperCW()
            if(data=="midStepperCCW"):
                midStepperCCW()
            if(data=="stepper3CW"):
                Stepper3CW()
            if(data=="stepper3CCW"):
                Stepper3CCW() 

            if(data=="servo_right"):                            #Servo motors control (The steering control)
                servo_right()
            if(data=="servo_left"):
                servo_left()
            #if(data=="trigger_dartgun"):
                #trigger_dartgun()
            if(data=="brake"):
                brake()
            else :
                direction=data[0:5]
                pwm=int(str[5:])
                if(direction=="Fward"):
                    forward(pwm)
                if(data=="Bward"):
                    backward(pwm)

    except KeyboardInterrupt :
            GPIO.cleanup()

