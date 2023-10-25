# -*- coding: utf-8 -*-
# Disciplina: Tópicos em Engenharia de Controle e Automação IV (ENG075): 
# Fundamentos de Veículos Autônomos - 2023/2
# Professores: Armando Alves Neto e Leonardo A. Mozelli
# Cursos: Engenharia de Controle e Automação
# DELT – Escola de Engenharia
# Universidade Federal de Minas Gerais
########################################

import class_car as cp
import numpy as np
import cv2
import time
import matplotlib.pyplot as plt
from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg as FigureCanvas 


########################################
# GLOBAIS
########################################
# parametros do carro
CAR = {
		'VELMAX'	: 5.0,				# m/s
		'ACCELMAX'	: 0.25, 			# m/s^2
		'STEERMAX'	: np.deg2rad(20.0),	# deg
	}


# mapa de plot
mapa = cv2.cvtColor(cv2.imread('./coppelia/pista.png'), cv2.COLOR_RGB2BGR)

def blob(image,min_hsv,max_hsv,color=(0,255,0)):
	hsv = cv2.cvtColor(image,cv2.COLOR_BGR2HSV) # Convert to hsv

	lower_color = np.array(min_hsv)
	upper_color = np.array(max_hsv)

	mask = cv2.inRange (hsv,lower_color,upper_color)

	contours,_ = cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)

	centers = []
	for i, contour in  enumerate(contours):
		area = cv2.contourArea(contour) # Calculate area
		M = cv2.moments(contour)
		if M["m00"] != 0:
			cx = int(M['m10'] / M["m00"])
			cy = int(M['m01'] / M["m00"])
			centers.append((cx,cy))

	return image, centers,mask

########################################
# cria comunicação com o carrinho
car = cp. CarCoppelia(mode='zmq')
car.stopMission()
time.sleep(0.3)


car.sim.setBoolParam(car.sim.boolparam_realtime_simulation,True) # Definir em tempo real (Equivalente ao clicar no "Relogio")
print("Iniciando Missão")
car.startMission()
car.sim.setInt32Param(car.sim.intparam_speedmodifier,6) # Equivalente ao clicar no "Coelho" 6 vezes

### QUESTAO 7
BLUE_HSV_MIN = [100,90,160]
BLUE_HSV_MAX = [110,130,200]
direction = 0 # -1 -> Esquerda, 1 -> Direita
while car.t < 140:
	# lê sensores
	car.step()

	# Pega a camera
	image = car.getImage()

	image = cv2.flip(cv2.cvtColor(image, cv2.COLOR_BGR2RGB), 0) # Inverte a imagem

	car.setSteer(0.037)

	car.setVel(1) # Somente para o carrinho andar

	# Identifica as janelas azuis
	image,centers,mask = blob(image,BLUE_HSV_MIN,BLUE_HSV_MAX, color=(0,255,0))


	if len(centers) > 0: # Caso detecte alguma janela azul
		media = np.average(np.array(centers), axis=0) # Calcual a media dos seus centros

		if media[0] < image.shape[1]/2: # Se a media,em x, dos centros janelas estiver no lado esquerdo da imagem
			direction = -1
		else:
			direction = 1


	if direction == -1:
		print("CASA ESTA A ESQUERDA")

	elif direction == 1:
		print("CASA ESTA A DIREITA")

	cv2.imshow("Original",image)
	cv2.imshow("Mascara",mask)

	if cv2.waitKey(1) & 0xFF == ord('q'):
		break