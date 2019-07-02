import RPi.GPIO as GPIO  
from time import sleep
import time
import Adafruit_GPIO.SPI as SPI
import Adafruit_MCP3008
ena = 16            
in1 = 20
in2 = 21
aux = 0
GPIO.setmode(GPIO.BCM)
GPIO.setup(ena,GPIO.OUT)
GPIO.setup(in1,GPIO.OUT)
GPIO.setup(in2, GPIO.OUT)
SPI_PORT   = 0
SPI_DEVICE = 0
pwm_a = GPIO.PWM(ena, 1000)
pwm_a.start(0)

mcp = Adafruit_MCP3008.MCP3008(spi=SPI.SpiDev(SPI_PORT, SPI_DEVICE))
class Contador(object):
	def __init__(self):  
		self.a = False
		self.b = False
		self.Kp = 25
		self.Ki = 0
		self.Kd = 23
		self.error_previo = 0.00
		self.error = 0.00
		self.integral = 0.00
		self.derivate = 0.00
		self.output = 0.00
		self.sp = 17
		self.lectura = 0 
		self.cm = 0
		#variables de kalman
		self.varsensor = 1.5E-05;  
		self.varProcess = 1e-7;
		self.P = 1.0;
		self.Pc = 0.0;
		self.G = 0.0;
		self.Xp = 0.0;
		self.Zp = 0.0;
		self.Xe = 0.0;
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
	elif cuenta.output < 50:
		cuenta.output=50
	cuenta.cm = pow(3027.4 / values[0], 1.2134);
	cuenta.Pc = cuenta.P + cuenta.varProcess;
	cuenta.G = cuenta.Pc/(cuenta.Pc + cuenta.varsensor);   
	cuenta.P = (1-cuenta.G)*cuenta.Pc;
	cuenta.Xp = cuenta.Xe;
	cuenta.Zp = cuenta.Xp;
	cuenta.Xe = cuenta.G*(cuenta.cm-cuenta.Zp)+cuenta.Xp;
	time.sleep(0.01)   
	#print(cuenta.Xe);
	print("Lectura: ",'%.2f'% cuenta.Xe ,"Error: ",cuenta.error,"Output: ",cuenta.output)	
	pwm_a.ChangeDutyCycle(cuenta.output)
	cuenta.error=cuenta.sp- int(cuenta.Xe)
	cuenta.integral=cuenta.integral+(cuenta.error*100)
	if cuenta.integral > 10:
 		cuenta.integral=10
	if cuenta.integral < -10:
 		cuenta.integral=-10
	if cuenta.error ==0:
 		cuenta.integral=0		
	cuenta.derivate=(cuenta.error-cuenta.error_previo)/100;
	cuenta.output=(cuenta.Kp*cuenta.error + cuenta.Ki*cuenta.integral+cuenta.Kd*cuenta.derivate)
	cuenta.error_previo=cuenta.error
	if cuenta.output < 0:
 		cuenta.output = cuenta.output * -1
	if cuenta.Xe<=(cuenta.sp+2) and cuenta.Xe>=(cuenta.sp-2):
		cuenta.a=False
		cuenta.b=False  
		cuenta.Giro_Favor_Reloj_MotorA()
	elif cuenta.Xe<(cuenta.sp-2):
		cuenta.a=False
		cuenta.b=True		
		cuenta.Giro_Favor_Reloj_MotorA()	
	elif cuenta.Xe>(cuenta.sp+2):
		cuenta.a=True
		cuenta.b=False
		cuenta.Giro_Favor_Reloj_MotorA()
	aux = 0

