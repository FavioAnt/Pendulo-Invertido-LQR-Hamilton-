import RPi.GPIO as GPIO  
from time import sleep
import time
import Adafruit_GPIO.SPI as SPI
import Adafruit_MCP3008
ena = 16            
in1 = 20
in2 = 21


GPIO.setmode(GPIO.BCM)
GPIO.setup(ena,GPIO.OUT)
GPIO.setup(in1,GPIO.OUT)
GPIO.setup(in2, GPIO.OUT)
SPI_PORT   = 0
SPI_DEVICE = 0
pwm_a = GPIO.PWM(ena, 600)
pwm_a.start(0)

mcp = Adafruit_MCP3008.MCP3008(spi=SPI.SpiDev(SPI_PORT, SPI_DEVICE))
class Contador(object):
	def __init__(self):
		self.numero = 0          
		self.count = 0           
		self.a = False
		self.b = False
		self.Kp = 50
		self.Ki = 1.5
		self.Kd = 25
		self.error_previo = 0
		self.error = 0
		self.integral = 0
		self.derivate = 0.00
		self.output = 0
		self.output1 = 0
		self.sp = 400
		
	def siguiente(self):
	        self.numero += 1       
	        return self.numero      
	def  Giro_Favor_Reloj_MotorA(self):
	        GPIO.output(in1,self.b)
	        GPIO.output(in2,self.a) 
cuenta = Contador()
print ("Program will finish after 100000 seconds or if you press CTRL+C\n")

while True:      
	
	values = [0]*8        
	for i in range(8):		 
		values[i] = mcp.read_adc(i)
	if cuenta.output > 99:
		cuenta.output=100
	print("Lectura: ",values[1],"Error: ",cuenta.error,"Output: ",cuenta.output)
	print(values[1])
	pwm_a.ChangeDutyCycle(cuenta.output)
	cuenta.error=cuenta.sp-values[1]
	cuenta.integral=cuenta.integral+(cuenta.error*0.5)
	if cuenta.integral > 10:
		cuenta.integral=10
	if cuenta.integral < -10:
		cuenta.integral=-10
	if cuenta.error ==0:
		cuenta.integral=0		
	cuenta.derivate=(cuenta.error-cuenta.error_previo)/0.5;
	cuenta.output=(cuenta.Kp*cuenta.error + cuenta.Ki*cuenta.integral+cuenta.Kd*cuenta.derivate)
	cuenta.error_previo=cuenta.error
	if cuenta.output < 0:
		cuenta.output = cuenta.output * -1		        
	cuenta.numero=0		
	cuenta.Giro_Favor_Reloj_MotorA()
	if values[1]<=cuenta.sp+2 and values[1]>=cuenta.sp-2:
		cuenta.a=False
		cuenta.b=False         
	elif values[1]<cuenta.sp+2:
		cuenta.a=False
		cuenta.b=True				
	elif values[1]>cuenta.sp-2:
		cuenta.a=True
		cuenta.b=False
	

			

