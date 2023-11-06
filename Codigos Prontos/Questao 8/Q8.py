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


def ControleLateral(car_x,line_x,line_psi,car_psi,k):
	# k = 1.2 #ganho
	# k = 0.2 #ganho COM VEL 0.3
	# k=1.2

	delta_max = np.deg2rad(20.0) #máximo de guinada

	# cálculo do erro e psi no caso da reta
	erro = (line_x-car_x) # menos a posição x do carro
	psierro = -(np.pi/2 - line_psi)
	# print(f'line_x:{line_x} / erro:{round(erro,5)} / psierro: {round(psierro,2)} /2o erro:{np.arctan2( k * erro , car.v)}')

	#Controle Stanley
	acao_guinada = psierro + np.arctan2( k  * erro , car.v)
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


	# Ajusta qual foi a ultima deteccao feita
	if ultima_deteccao == 0 and detecta_placa_verde:
		ultima_deteccao = 1
	elif ultima_deteccao == 1 and detecta_placa_vermelha:
		ultima_deteccao = 2


	if ultima_deteccao == 0:
		car.setVel(0.7)
		delta,erro,psierro = ControleLateral(car_x=car.p[0],line_x=0,line_psi=car.th,car_psi=0,k=0.2)
	elif ultima_deteccao == 1:
		car.setVel(1.0)
		delta,erro,psierro = ControleLateral(car_x=car.p[0],line_x=0,line_psi=car.th,car_psi=0,k=0.8)
	elif ultima_deteccao == 2:
		car.setVel(1.5)
		delta,erro,psierro = ControleLateral(car_x=car.p[0],line_x=0,line_psi=car.th,car_psi=0,k=1.0)

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
df.to_csv('Q8.csv', index=True)
