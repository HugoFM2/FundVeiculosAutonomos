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
import matplotlib.pyplot as plt


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

#Traçar pontos da trajetória
def trajetoria (x0, y0, raio):
	num_pontos = 10
	vetores = []
	
    #Trecho reto da origem até tangente do primeiro círculo à esquerda
	for i in range (num_pontos-1):
		x = x0
		y = y0 + i*(raio/num_pontos)
		theta = np.pi/2
		vetores.append((x, y, theta))
	
    #Trecho circular da 0 a 3pi/2 do circulo à esquerda (sentido antihorário)
	angulos = np.linspace(0, 3*np.pi/2 , num_pontos)
	
	for angulo in angulos:
		x = raio * np.cos(angulo) - raio
		y = raio * np.sin(angulo)  + raio
		
		theta = angulo + np.pi/2
		theta = theta % (2 * np.pi)
		
		vetores.append((x, y, theta))
		
	#Trecho reto do círculo à esquerda até origem
	for i in range (num_pontos-1):
		x = x0 - raio + i*(raio/num_pontos)
		y = y0
		theta = 0.0
		vetores.append((x, y, theta))
	
    #Trecho reto da origem até o círculo à direita
	for i in range (num_pontos-1):
		x = x0 + i*(raio/num_pontos)
		y = y0
		theta = 0.0
		vetores.append((x, y, theta))			

    #Trecho circular de pi/2 a pi do circulo à direita (sentido horário)	
	for angulo in angulos:
		x = raio * np.cos((np.pi/2)-angulo) + raio
		y = raio * np.sin((np.pi/2)-angulo) - raio
		
		theta = (2*np.pi)-angulo
		theta = theta % (2 * np.pi)
		vetores.append((x, y, theta))
    
	return vetores
    

        

#Parâmetros do carro
L = 0.30 # coprimento do carro em metros
delta_max = np.deg2rad(20.0) #máximo de guinada

#Parâmetros do controlador
k = 0.8 # ganho
delta = 0.0 # inicial

###################################################
#Parâmetros da trajetoria (Defina: círculo centro (x0,y0) e raio=raio)

x0 = 0.0
y0 = 0.0
r = 2
pontos_trajetoria = trajetoria (x0 , y0 , r)
index = 0


while car.t < 100.0:
	
	# lê sesnores
	car.step()

	# Pega a camera
	image = car.getImage()

		
    ################################################################
    # Ponto a ser seguido	
	ponto_atual = pontos_trajetoria[index]
	
	#################################################################
    # Cálculo de psi e erro
	if  np.abs (car.th - ponto_atual[2]) > np.pi: ## tratamento erro numerico - quando car.th 1Q e orientacao 4Q
		psi = car.th - ponto_atual[2] + 2*np.pi
	else:	
		psi = car.th - ponto_atual[2]
		 
	
	erro = dist_cart(ponto_atual[0], ponto_atual[1], car.p[0], car.p[1]) # distancia do carro para um círculo de raio centrado em x0,y0
	##############################################################################
    ##### ISSO PRECISA SER RESOLVIDO ( QUAL O SINAL DO ERRO)
	# if psi > 0:
	#  	erro = -erro # necessário inverter sinal pois o erro foi calculado de forma escalar
    
    ###################################################################
	#######################################################################	

	#Controle Stanley
	acao_guinada = psi + np.arctan2(( k * erro ), car.v)
	if np.abs(acao_guinada) < delta_max: 
		delta = acao_guinada 		
	else:
		if acao_guinada >= delta_max:#saturou positivamente
			delta = delta_max
		else: # saturou negativamente acao_guinada <= -delta_max:
			delta = -delta_max

    
	#print(ponto_atual)
	#print(erro, psi, car.th)

	# atua
	car.setSteer(delta)
	
	car.setVel(0.8) # Controle de velocidade basico 
	##########################################################################################

    ###################################################################
   #Caso erro menor que o determinado, passa para próximo ponto	
	while(np.abs(erro) < 0.1 and np.abs(psi) < 0.1 ):
		index = (index + 1) % np.size(pontos_trajetoria)
		erro = dist_cart(ponto_atual[0], ponto_atual[1], car.p[0], car.p[1])

    ###################################################################
    # lê e exibe camera
	
	img = cv2.flip(cv2.cvtColor(image, cv2.COLOR_BGR2RGB), 0)
	cv2.imshow("Camera",img)
	if cv2.waitKey(1) & 0xFF == ord('q'):
		break
	
print('Terminou...')
