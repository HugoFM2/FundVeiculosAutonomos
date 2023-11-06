# -*- coding: utf-8 -*-
# Disciplina: Tópicos em Engenharia de Controle e Automação IV (ENG075): 
# Fundamentos de Veículos Autônomos - 2023/2
# Professores: Armando Alves Neto e Leonardo A. Mozelli
# Cursos: Engenharia de Controle e Automação
# DELT – Escola de Engenharia
# Universidade Federal de Minas Gerais
########################################
import sys
sys.path.append("../")
sys.path.append("../coppelia/")
sys.path.append("../coppeliasim_zmqremoteapi/")
import class_car as cp
import numpy as np
import cv2
import time, math
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
erros = []
psierros = []
vs = []
steers = []

theta = 0.0
rho = 0.0

GREEN_HSV_MIN =  [40,200,40] # PISTA CIRCUITO
GREEN_HSV_MAX = [70,255,255] # PISTA CIRCUITO

RED_HSV_MIN =  [0,150,80] # PISTA CIRCUITO
RED_HSV_MAX = [20,255,255] # PISTA CIRCUITO




def blob(image,min_hsv,max_hsv,color=(0,255,0)):
	# Detecta blobs em HSV e retorna sua mascara
	hsv = cv2.cvtColor(image,cv2.COLOR_BGR2HSV) # Convert to hsv

	lower_color = np.array(min_hsv)
	upper_color = np.array(max_hsv)

	mask = cv2.inRange (hsv,lower_color,upper_color)

	return mask

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
	# print(f"x1:{x1}/y1:{y1}/x2:{x2}/y2:{y2}")
	cv2.line(image, (x1, y1), (x2, y2), (255, 0, 0), 2)  # Desenha a linha detectada
	# Tratamento de x2-x1 = 0
	if (x2-x1) != 0:
		m = (y2 - y1) / (x2 - x1)
	else:
		m = (y2-y1) / 0.00000001
	b_constant = y1 - m * x1

	if m == 0:
		m = 0.000001


	x_line = (mask.shape[0] - b_constant) / m # Ponto em que y = 300

	# Desenha a linha central do modelo (Apenas para debug) # ATENCAO, O EIXO 0,0 É O CANTO SUPERIOR ESQUERDO DA IMAGEM
	x1_car = int(mask.shape[1]/2) # 300
	y1_car = int(mask.shape[0]) # 600, a parte inferior
	x2_car = int(mask.shape[1]/2) #300
	y2_car = int(0) # 0 (O topo)
	# cv2.line(mask, (x1_car, y1_car), (x2_car, y2_car), (255, 255, 255), 2)  # Draw lines in red
	cv2.line(image, (x1_car, y1_car), (x2_car, y2_car), (0, 255, 0), 2)  # Desenha linha do carro


	# if showImage:
	cv2.imshow("Linhas", mask)
	angulo = math.atan(m)
	# print(f'y0:{y0} / rho: {rho} / a: {a} / b:{b} / X_LINE:{x_line}')
	print("angulo:",angulo - np.pi/2)

	return -angulo,x_line


def detectPlaca(image,HSV_MIN,HSV_MAX):
	image_test = image.copy()

	# Detecta a Area Verde (sua mascara)
	mask_color = blob(image_test,HSV_MIN,HSV_MAX, color=(255,255,255))

	# Encontra os contornos da area 
	contours,_ = cv2.findContours(mask_color,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)



	for contour in contours:
		area = cv2.contourArea(contour) # Calcula a area
		# print(area)

		# Detecta se é um hexagono (Faz uma aproximacao do contorno)
		peri = cv2.arcLength(contour, True)
		approx = cv2.approxPolyDP(contour, 0.02 * peri, True)
		# cv2.drawContours(image, [contour], -1, (0,255,255), 3)
		if len(approx) == 8 and area > 700.0: # Caso detecte o octogono e esteja perto o suficiente, retornar true
			cv2.drawContours(image, [contour], -1, (255,255,0), 5) 
			return True,mask_color


	# Caso nao ache o octogono, retorna falso
	# cv2.imshow('GreenMask',mask_green)

	return False,mask_color


def ControleLateralVisao(car_x,line_x,line_psi,line_rho,car_psi,k=1.0):
	# k = 0.3 #ganho
	ke = 0.00392 # Multiplicador do erro, transforma pixels para m
	delta_max = np.deg2rad(20.0) #máximo de guinada

	# cálculo do erro e psi no caso da reta
	erro = ke*(line_x-car_x) # menos a posição x do carro
	#psierro = -((line_psi)-np.arctan2(erro, 1))
	psierro = line_psi
	if psierro < 0:
		psierro += np.pi/2
	else:
		psierro -=np.pi/2


	# psierro = 0.0
	# print(f'erro:{round(np.arctan(( k * erro) / car.v),5)} / psierro: {round(psierro,2)}')
	print(f'line_psi:{round(line_psi,5)} / phi: {(np.arctan2(erro, 1))}')

	#Controle Stanley
	acao_guinada = psierro + np.arctan(( k * erro) / car.v)
	if np.abs(acao_guinada) < delta_max: 
	    delta = acao_guinada 		
	else:
		if acao_guinada >= delta_max:#saturou positivamente
			delta = delta_max
		else: # saturou negativamente acao_guinada <= -delta_max:
			delta = -delta_max

	return delta,erro,psierro





ultima_deteccao = 0 # 0 para nenhuma placa detectada, 1 para placa verde detectada, 2 para placa vermelha detectada
while car.t < 65:
	
	# lê sensores
	car.step()

	# Pega a camera
	image = car.getImage()

	image = cv2.flip(cv2.cvtColor(image, cv2.COLOR_BGR2RGB), 0) # Inverte a imagem




	# Obtem a detecção das placas verde e vermelha
	detecta_placa_verde,mask_verde = detectPlaca(image,GREEN_HSV_MIN,GREEN_HSV_MAX)
	# print(f'Detectou Placa Verde:{detecta_placa_verde}')

	detecta_placa_vermelha, mask_vermelha =  detectPlaca(image,RED_HSV_MIN,RED_HSV_MAX)
	# print(f'Detectou Placa Vermelha:{detecta_placa_verde}')
	# cv2.imshow('PlacaVermelha',mask_vermelha)


	# Obtem a inclinação e posicao da linha
	theta,line_x = getDashedLine(image)


	# Ajusta qual foi a ultima deteccao feita
	if ultima_deteccao == 0 and detecta_placa_verde:
		ultima_deteccao = 1
	elif ultima_deteccao == 1 and detecta_placa_vermelha:
		ultima_deteccao = 2


	if ultima_deteccao == 0:
		car.setVel(0.5)
	elif ultima_deteccao == 1:
		car.setVel(1.0)
	elif ultima_deteccao == 2:
		car.setVel(1.5)

	delta,erro,psierro = ControleLateralVisao(car_x=300,line_x=line_x,line_psi=theta, line_rho=rho,car_psi=0,k=0.1)
	car.setSteer(delta) 




	cv2.imshow("Camera",image)


	# Faz o controle lateral baseado nas posicoes relativas do carrinho
	car.setSteer(delta) 




	# Armazena valores de posicao e tempo do carrinho
	x.append(round(car.p[0],4))
	y.append(round(car.p[1],4))
	time.append(round(car.t,4))
	erros.append(round(erro,4))
	psierros.append(round(psierro,4))
	vs.append(round(car.v,4))
	steers.append(round(delta,4))




	if cv2.waitKey(1) & 0xFF == ord('q'):
		break
	
print('Terminou...')

# Salva os dados em um formato csv
import pandas as pd
df = pd.DataFrame({'ts'    : time,
					'x'   : x,
					'y'   : y,
					'erros' :erros,
					'psierros' : psierros,
					'vel'    : vs,
					'steers' : steers})
df.to_csv('Q9.csv', index=True)
