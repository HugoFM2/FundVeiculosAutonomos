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


########################################
# cria comunicação com o carrinho
car = cp. CarCoppelia()
car.sim.setBoolParam(car.sim.boolparam_realtime_simulation,True) # Definir em tempo real (Equivalente ao clicar no "Relogio")

car.startMission()
car.sim.setInt32Param(car.sim.intparam_speedmodifier,6) # Equivalente ao clicar no "Coelho" 6 vezes

x= []
y = []
time = []

# #Calcula distância entre dois pontos cartesianos
# def dist_cart (x0, y0, xf, yf):
# 	return np.sqrt((xf - x0)**2 + (yf - yf)**2)

#Parâmetros do carro
L = 0.30 # coprimento do carro em metros
delta_max = np.deg2rad(20.0) #máximo de guinada

#Parâmetros do controlador
k = 0.5 #ganho


while car.t < 40.0:
	
	# lê sesnores
	car.step()

	# Pega a camera
	image = car.getImage()

	# cálculo do erro e psi no caso da reta
	erro = -car.p[0] # menos a posição x do carro
	psi = -(np.pi/2 - car.th) # pi/2 orientação

	#Controle Stanley
	acao_guinada = psi + np.arctan(( k * erro) / car.v)
	if np.abs(acao_guinada) < delta_max: 
	    delta = acao_guinada 		
	else:
		if acao_guinada >= delta_max:#saturou positivamente
			delta = delta_max
		else: # saturou negativamente acao_guinada <= -delta_max:
			delta = -delta_max

    
	print(car.p[0], car.th)
	# atua
	car.setSteer(delta) 
	car.setVel(2.0) # Controle de velocidade basico (cte a 2 m/s)
	
	#Adiciona os valores aos vetores
	x.append(car.p[0])
	y.append(car.p[0])
	time.append(car.t)


	# lê e exibe camera
	img = cv2.flip(cv2.cvtColor(image, cv2.COLOR_BGR2RGB), 0)
	cv2.imshow("Camera",img)
	if cv2.waitKey(1) & 0xFF == ord('q'):
		break
	
print('Terminou...')

# Salva os dados em um formato csv
import pandas as pd
df = pd.DataFrame({'ts'    : time,
					'x'   : x,
					'y'   : y})
df.to_csv('Dados_Stanley.csv', index=True)