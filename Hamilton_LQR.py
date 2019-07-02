import RPi.GPIO as GPIO  
from time import sleep
import time
import Adafruit_GPIO.SPI as SPI
import Adafruit_MCP3008
import math
import numpy
import scipy.linalg
import numpy as np
#Puente H
ena = 12            
in1 = 20
in2 = 21
#Encoder Pendulo
clk = 18
dt = 23
#Puente H
GPIO.setmode(GPIO.BCM)
GPIO.setup(ena,GPIO.OUT)
GPIO.setup(in1,GPIO.OUT)
GPIO.setup(in2,GPIO.OUT)
SPI_PORT   = 0
SPI_DEVICE = 0
pwm_a = GPIO.PWM(ena, 90)
pwm_a.start(0)
#MCP 3208
mcp = Adafruit_MCP3008.MCP3008(spi=SPI.SpiDev(SPI_PORT, SPI_DEVICE))
#Encoder Pendulo
GPIO.setmode(GPIO.BCM)
GPIO.setup(clk, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(dt, GPIO.IN, pull_up_down=GPIO.PUD_UP)
clkLastState = GPIO.input(clk)
class hamilton(object):
	def __init__(self): 
		#Variables filtro Kalman 
		self.varsensor = 1.5E-05  
		self.varProcess = 1e-7
		self.P = 1.0
		self.Pc = 0.0
		self.G = 0.0
		self.Xp = 0.0
		self.Zp = 0.0
		self.Xe = 0.0
		#Espacio de estados
		self.M=0.090;
		self.m=0.042;
		self.l=0.15;
		self.j=0.099;
		self.g=9.78;
		self.b=0.05;
		self.si= self.M*self.m*(self.l**2)+self.j*(self.M+self.m)
		self.x1= ((self.j+self.m*(self.l**2))*self.b/self.si)
		self.x2= ((self.m**2*self.l**2*self.g)/self.si)
		self.x6= ((self.m*self.l)/self.si)
		self.x3 = -(self.m*self.l*self.b)/(self.si)
		self.x4 = ((self.m*self.g*self.l*(self.M+self.m))/(self.si))
		self.x5 = ((self.j+self.m*self.l**2)/self.si)
		self.A = matrix ( [ [ 0 , 1 , 0 , 0 ] , [ 0 , self.x1 , self.x2 , 0 ] , [ 0 , 0 , 0 , 1 ] , [ 0 , self.x3 , self.x4 , 0 ] ] )
		self.B = matrix ( [ [0] , [self.x5] , [0] , [self.x6] ] )
		self.C = matrix ( [ [ 1 , 0 , 0 , 0 ] , [ 0 , 0 , 1 , 0 ] ] )
		#Matrices de pesado
		self.Q = matrix ( [ [ 1 , 0 , 0 , 0 ] , [ 0 , 0 , 0 , 0 ] , [ 0 , 0 , 8490 , 0 ] , [ 0 , 0 , 0 , 0 ] ] )
		#self.Q = self.C.I * self.C
		self.R = 1
		self.Kaux = matrix ([[ -1.00 , -13.15 , 99.07 , 5.17 ]])		
		#Ganancia
		self.K = self.Kaux	
	def kalman(self, kalm):		
		self.Pc = self.P + self.varProcess;
		self.G = self.Pc/(self.Pc + self.varsensor)
		self.P = (1-self.G)*self.Pc;
		self.Xp = self.Xe;
		self.Zp = self.Xp;
		self.Xe = self.G*(kalm-self.Zp)+self.Xp;		
	def  Giro_Motor_Izquierda(self):
		GPIO.output(in1,False)
		GPIO.output(in2,True)		
	def  Giro_Motor_Derecha(self):
		GPIO.output(in1,True)
		GPIO.output(in2,False)	
	def  Giro_Motor_Off(self):
		GPIO.output(in1,False)
		GPIO.output(in2,False)	
Hamilton = hamilton()
spa=90
spd=110
counter=0
print ("Iniciando Hamilton presione (CTRL+C) para detener el programa\n")
try:
	while True:		
		values = [0]*4	      
		values[0] = mcp.read_adc(0)	
		values[1] = mcp.read_adc(1)	
		values[2] = ((mcp.read_adc(2)*100/1023)-100)*(300/(-100))
		centro=values[1]
		values[2]=abs (values[2])
		v=(values[0]/1023.0)*3.3
		distancia = 16.2537 * v**4 - 129.893 * v**3 + 382.268 * v**2 - 512.611 * v + 301.439	
		Hamilton.kalman(distancia)	
		if (centro!=1023):
			counter=spa	
		clkState = GPIO.input(clk)		
		if clkState != clkLastState:
			dtState = GPIO.input(dt)
			if dtState != clkState:
				counter += 1
			else:
				counter -= 1
		clkLastState = clkState	
		if( counter>spa):	
			Hamilton.Giro_Motor_Derecha()		
		if( counter<spa):	
			Hamilton.Giro_Motor_Izquierda()
		if ( counter==spa):
			if( int(Hamilton.Xe)>spd):	
				Hamilton.Giro_Motor_Derecha()		
			if( int(Hamilton.Xe)<spd):	
				Hamilton.Giro_Motor_Izquierda()
		X = matrix ([[int(Hamilton.Xe)] , [0] , [counter] , [0]])
		ref = matrix ([[spd] , [0] , [spa] , [0]])
		u=-Hamilton.K*(X-ref)
		print ("u: ",u,"Angulo: ",counter, "Distancia: ", Hamilton.Xe, "Centro: ", centro)	
		if(u<=0):	
			u=u*-1
		if(u>=100):	
			u=100
		pwm_a.ChangeDutyCycle(u)		
finally:
        GPIO.cleanup()        
