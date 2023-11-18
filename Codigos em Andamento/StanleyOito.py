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


########################################
# cria comunicação com o carrinho
car = cp. CarCoppelia()
car.sim.setBoolParam(car.sim.boolparam_realtime_simulation,True) # Definir em tempo real (Equivalente ao clicar no "Relogio")

car.startMission()
car.sim.setInt32Param(car.sim.intparam_speedmodifier,6) # Equivalente ao clicar no "Coelho" 6 vezes

x= []
y = []
time = []

#Calcula distância entre dois pontos cartesianos
def dist_cart (x0, y0, xf, yf):
	return np.sqrt((xf - x0)**2 + (yf - y0)**2)

### DEFININDO PONTOS DO INFINITO E ORIENTACAO
amplitude = 8
frequency = 0.5  
num_points = 100 


t = np.linspace(0, 4 * np.pi, num_points)


xs = amplitude * np.sin(frequency*t)
ys = amplitude * np.sin(2*frequency*t)

# # Calcula Orientacao
# slopes = []
# for i in range(len(x) -1):
# 	vector_x = x[i+1] - x[i]
# 	vector_y = y[i+1] - y[i]
# 	theta = math.atan2(vector_y,vector_x)
# 	slopes.append(theta)

#Parâmetros do carro
L = 0.30 # coprimento do carro em metros
delta_max = np.deg2rad(20.0) #máximo de guinada

#Parâmetros do controlador
k = 0.8 #ganho
delta = 0.0

#Parâmetros da trajetoria (Defina: círculo centro (x0,y0) e raio=raio)
orientacao = 0.0
x0 = -1.0
y0 = -5.0
raio = 2.0
sentido = 1 # 1 para sentido anti-horário e 3 para sentido horário


while car.t < 100.0:
	
	# lê sesnores
	car.step()

	# Pega a camera
	image = car.getImage()

	# ponto na curva com a menor distancia
	dist = [np.linalg.norm(car.p - np.array([xs[i], ys[i]])) for i in range(len(xs))]
	idx = np.argmin(dist)


	#Calcula a referencia
	xc, yc = car.p
	X1, Y1 = xs[idx], ys[idx]
	X2, Y2 = xs[idx+1], ys[idx+1]
	PSIREF = np.arctan2(Y2-Y1, X2-X1)



	# # for x,y in zip(xs,ys):
	# ################################################################
	# # Gerando a trajetória de um círculo centrado em x0, y0
	# orientacao = np.arctan2(car.p[1]-y0, car.p[0]-x0)
	# if orientacao > 0:
	# 	orientacao =orientacao +  np.pi/2 * sentido 
		
	# else: # no caso de estar no 3º ou 4º quadrante  arctan é negativo
	# 	orientacao = orientacao + 2*np.pi  + np.pi/2 * sentido 
	
	# orientacao = orientacao % (2 * np.pi)
	
	#################################################################
	# Cálculo de psi e erro
	psie = PSIREF - car.th
	# while psie > np.pi:
	# 	psie -= 2.0*np.pi
	# while psie < -np.pi:
	# 	psie += 2.0*np.pi

	# if  np.abs (car.th - orientacao) > np.pi: ## tratamento erro numerico - quando car.th 1Q e orientacao 4Q
	# # if (orientacao >= np.pi and car.th <= np.pi):
	# 	psi = car.th - orientacao + 2*np.pi
	# else:	
	# 	psi = car.th - orientacao
		 
	
	erro = raio - dist_cart(xs[idx], ys[idx], car.p[0], car.p[1]) # distancia do carro para um círculo de raio centrado em x0,y0
	if sentido == 3:
		erro = -erro # necessário inverter sinal pois o erro foi calculado de forma escalar
		

	#Controle Stanley
	acao_guinada = psie + np.arctan2(( k * erro ), car.v)
	if np.abs(acao_guinada) < delta_max: 
		delta = acao_guinada 		
	else:
		if acao_guinada >= delta_max:#saturou positivamente
			delta = delta_max
		else: # saturou negativamente acao_guinada <= -delta_max:
			delta = -delta_max

	
	# print(car.p[0], car.th, orientacao)
	print(f'')
	print(f'erro: {round(erro,2)} / dist_car: {round(dist_cart(xs[idx], ys[idx], car.p[0], car.p[1]),2)} / psierro:{round(psie,2)} / xc: {round(xc,2)} / x1: {round(X1,2)}')

	# atua
	car.setSteer(delta)
	
	car.setVel(0.8) # Controle de velocidade basico 
	
	# lê e exibe camera
	
	img = cv2.flip(cv2.cvtColor(image, cv2.COLOR_BGR2RGB), 0)
	cv2.imshow("Camera",img)
	if cv2.waitKey(1) & 0xFF == ord('q'):
		break
	
print('Terminou...')