# -*- coding: utf-8 -*-
# Disciplina: Tópicos em Engenharia de Controle e Automação IV (ENG075): 
# Fundamentos de Veículos Autônomos - 2023/2
# Professores: Armando Alves Neto e Leonardo A. Mozelli
# Cursos: Engenharia de Controle e Automação
# DELT – Escola de Engenharia
# Universidade Federal de Minas Gerais
########################################
import sys
sys.path.append("../../")
sys.path.append("../../coppelia/")
sys.path.append("../../coppeliasim_zmqremoteapi/")

import class_car as cp
import numpy as np
import cv2
import time
import matplotlib.pyplot as plt
from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg as FigureCanvas 

########################################
# cria comunicação com o carrinho
car = cp. CarCoppelia()
car.stopMission()
time.sleep(0.3)


car.sim.setBoolParam(car.sim.boolparam_realtime_simulation,True) # Definir em tempo real (Equivalente ao clicar no "Relogio")

car.startMission()
car.sim.setInt32Param(car.sim.intparam_speedmodifier,6) # Equivalente ao clicar no "Coelho" 6 vezes

x= []
y = []
time = []

theta = 0
rho = 200
def getDashedLine(image):
	# Obtem a linha tracejada e retorna os valores de x1 e rho(Inclinacao)
	global theta,rho
	img = image.copy()


	kernel1 = np.ones((1,1),np.uint8)


	imgGray=cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
	# cv2.imshow("imgGray", imgGray) 
	imgBW=cv2.threshold(imgGray, 160, 255, cv2.THRESH_BINARY_INV)[1]
	

	img1=cv2.dilate(imgBW, kernel1, iterations=2)
	img1= cv2.bitwise_not(img1)
	# cv2.imshow("img1", img1) 


	contours,_ = cv2.findContours(img1,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)

	# largest_contours = sorted(contours, key=cv2.contourArea)
	# Filtra os contornos por área
	selected_contours = []
	for contour in contours:
		area = cv2.contourArea(contour)
		# print(area)
		if area < 600:
			selected_contours.append(contour)

	# Desenha os contornos em uma imagem nova com fundo preto apenas para detectar as linhas do contorno obtidas
	mask = np.zeros(img.shape, np.uint8)
	cv2.drawContours(mask, selected_contours, -1, (255,255,255,255), -1)
	

	# calcula Hough Line Transform
	edges = cv2.Canny(mask, 50, 155)
	lines = cv2.HoughLines(edges, 1, np.pi/180, threshold=40, min_theta=np.radians(-60), max_theta=np.radians(60))  # Adjust threshold as needed

	# desenha linhas e calcula a media delas
	linhas = []
	if lines is not None:
		for rho, theta in lines[:, 0]:
			if theta > np.pi/2:
				continue
			linhas.append( (theta,rho) )
		
		# Faz a media de theta e rho obtidos
		theta = sum([x[0] for x in linhas])/len(linhas)
		rho = sum([x[1] for x in linhas])/len(linhas)


	#Theta corresponde ao psi e o rho corresponde ao x
	a = np.cos(theta)
	b = np.sin(theta)
	x0 = a * rho
	y0 = b * rho
	x1 = int(x0 + 1000 * (-b))
	y1 = int(y0 + 1000 * (a))
	x2 = int(x0 - 1000 * (-b))
	y2 = int(y0 - 1000 * (a))
	cv2.line(image, (x1, y1), (x2, y2), (255, 0, 0), 2)  # Desenha a curva do carro


	# Desenha a linha central do modelo
	x1_car = int(image.shape[1]/2)
	y1_car = int(0)
	x2_car = int(image.shape[1]/2)
	y2_car = int(image.shape[0])
	cv2.line(image, (x1_car, y1_car), (x2_car, y2_car), (0, 255, 0), 2)  # Draw lines in red
	return theta,x1


def ControleLateral(car_x,line_x,line_psi,car_psi):
	k = 0.3 #ganho
	ke = 0.001 # Multiplicador do erro
	delta_max = np.deg2rad(20.0) #máximo de guinada

	# cálculo do erro e psi no caso da reta
	erro = ke*(line_x-car_x) # menos a posição x do carro
	psierro = -(line_psi) 
	# print(f'erro:{round(erro,5)} / psierro: {round(psierro,2)}')

	#Controle Stanley
	acao_guinada = psierro + np.arctan(( k * erro) / car.v)
	if np.abs(acao_guinada) < delta_max: 
	    delta = acao_guinada 		
	else:
		if acao_guinada >= delta_max:#saturou positivamente
			delta = delta_max
		else: # saturou negativamente acao_guinada <= -delta_max:
			delta = -delta_max

	return delta



while car.t < 40:
	# lê sesnores
	car.step()

	# Pega a camera
	image = car.getImage()
	image = cv2.flip(cv2.cvtColor(image, cv2.COLOR_BGR2RGB), 0) # Inverte a imagem

	# Armazena valores de posicao e tempo do carrinho
	x.append(car.p[0])
	y.append(car.p[1])
	time.append(car.t)


	# Obtem a inclinação e posicao da linha
	theta,line_x = getDashedLine(image)

	delta = ControleLateral(300,line_x=line_x,line_psi=theta,car_psi=0)


	car.setSteer(delta) 
	car.setVel(1)

	# lê e exibe camera
	cv2.imshow("Camera",image)

	x.append(car.p[0])
	y.append(car.p[0])
	time.append(car.t)
	


	if cv2.waitKey(1) & 0xFF == ord('q'):
		break
	
print('Terminou...')

# Salva os dados em um formato csv
import pandas as pd
df = pd.DataFrame({'ts'    : time,
					'x'   : x,
					'y'   : y})
df.to_csv('Q7.csv', index=True)