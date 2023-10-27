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


num_pontos = 50
num_pontos_reta = 50
#Traçar pontos da trajetória
def trajetoria (x0, y0, raio):
	# num_pontos = 50

	
	vetores = []
	
    #Trecho reto da origem até tangente do primeiro círculo à esquerda
	for i in range (num_pontos_reta):
		x = x0
		y = y0 + i*(raio/num_pontos_reta)
		theta = np.pi/2
		vetores.append((x, y, theta))
	
    #Trecho circular da 0 a 3pi/2 do circulo à esquerda (sentido antihorário)
	angulos = np.linspace(0, 3*np.pi/2 , num_pontos)

	for angulo in angulos:
		x = raio * np.cos(angulo) - raio + x0
		y = raio * np.sin(angulo)  + raio + y0
		
		theta = angulo + np.pi/2
		theta = theta % (2 * np.pi)
		
		vetores.append((x, y, theta))
		
	#Trecho reto do círculo à esquerda até origem
	for i in range (num_pontos_reta-1):
		x = x0 - raio + i*(raio/num_pontos_reta)
		y = y0
		theta = 0.0
		vetores.append((x, y, theta))
	
    #Trecho reto da origem até o círculo à direita
	for i in range (num_pontos_reta-1):
		x = x0 + i*(raio/num_pontos_reta)
		y = y0
		theta = 0.0
		vetores.append((x, y, theta))			

    #Trecho circular de pi/2 a pi do circulo à direita (sentido horário)	
	for angulo in angulos:
		x = raio * np.cos((np.pi/2)-angulo) + raio +x0
		y = raio * np.sin((np.pi/2)-angulo) - raio +y0
		
		theta = (2*np.pi)-angulo
		theta = theta % (2 * np.pi)
		vetores.append((x, y, theta))
	
    #Trecho reto da origem até tangente do primeiro círculo à esquerda
	for j in range (num_pontos_reta-1):
		x = x0 
		y = y0 + j*(raio/num_pontos_reta) - raio
		theta = np.pi/2
		vetores.append((x, y, theta))
    
	return vetores
    

        

#Parâmetros do carro
L = 0.30 # coprimento do carro em metros
delta_max = np.deg2rad(20.0) #máximo de guinada

#Parâmetros do controlador
k =  0.5# ganho
delta = 0.0 # inicial
min = 0.01
###################################################
#Parâmetros da trajetoria (Defina: círculo centro (x0,y0) e raio=raio)
px = []
py = []
x0 = 0.0
y0 = -3.0
r = 2
pontos_trajetoria = trajetoria (x0 , y0 , r)
index = 0
idx = 0
#print (pontos_trajetoria)

px = [i[0] for i in pontos_trajetoria]
py = [i[1] for i in pontos_trajetoria]
ptheta = [i[2] for i in pontos_trajetoria]
plt.figure()
plt.plot (px, py)
plt.show()
distancias_curva = []

# for i in range (595):
# 	print (i, "-",round (px[i], 2), round(py[i],2), round(ptheta[i],2))

primeiro = (True)

# mapa de chao
mapa = cv2.cvtColor(cv2.imread('./coppelia/pista.png'), cv2.COLOR_RGB2BGR)



while car.t < 30.0:
	
	# lê sesnores
	car.step()

	# Pega a camera
	image = car.getImage()

		
    ################################################################
    # Ponto a ser seguido	
	ponto_atual = pontos_trajetoria[index]
	
	#################################################################
    # Cálculo de psi e erro
	distancias_curva = []
	

	#Distância do carro a todos os posntos da curva
	for i in range (len(pontos_trajetoria)): 
		aux = dist_cart(px[i], py[i], car.p[0], car.p[1])
		distancias_curva.append(aux)
	idx = np.argmin(distancias_curva)
	
	if primeiro:
		idx = 2
		idx_ant = idx
		theta_anterior = car.t #para um filtro de car.t
		primeiro = False
	i=0

	
	# if  np.abs (ptheta[idx]-ptheta[idx_ant]) > np.pi: ## tratamento erro numerico - quando car.th 1Q e orientacao 4Q
	# 	aux_theta = (ptheta[idx]-ptheta[idx_ant] ) + 2*np.pi
		
	# else:	
	# 	aux_theta = (ptheta[idx]-ptheta[idx_ant])

	# while (aux_theta > np.pi/4):
	# 	if idx >idx_ant:
	# 		idx = np.argpartition(distancias_curva, i)[i]
	# 	i+=1
	# 	if i>=len(distancias_curva):
	# 		break
	#####
	while (idx < idx_ant or idx > idx_ant+2):
		idx = np.argpartition(distancias_curva, i)[i]
		i+=1
		if i>=len(distancias_curva):
			break
	
	idx_ant = idx
	if idx > (0.95*(2*num_pontos+4*num_pontos_reta)): #Completou mais que 95% do percurso
		idx = 1
		idx_ant =0

	erro = dist_cart(px[idx], py[idx], car.p[0], car.p[1])
	#################################
	## calculo erro do artigo
	################################
	# front_axle_vec = [-np.cos(car.th + np.pi / 2),
    #                   -np.sin(car.th + np.pi / 2)]
	# erro = -np.dot([px[idx]-car.p[0], py[idx]-car.p[1]], front_axle_vec)
    

	############################
	#Inversão do sinal de erro quando inicia a segunda curva
	###########################
	if idx<(num_pontos+2*num_pontos_reta) or idx>(0.9*(2*num_pontos+4*num_pontos_reta)):
		erro = -erro
	else:
		erro =  erro
	
	#############################
	#Calculo de PSI
	##########################
	if  np.abs (car.th - ptheta[idx]) > np.pi: ## tratamento erro numerico - quando car.th 1Q e orientacao 4Q
		psi = (car.th -ptheta[idx]) - 2*np.pi
		
	else:	
		psi = (car.th - ptheta[idx])

	##### Tentei colocar um filtro no car.t porque aparentemente ele está causando as oscilaçoes
	##### car.t  vem da classe do carro 

	# if (np.abs(theta_anterior - car.th)>np.pi):
	# 	theta = theta_anterior
	# else:
	# 	theta = car.th
	# theta_anterior = theta

	# psi = (car.th - ptheta[idx])
	# while psi > np.pi:
	# 	psi -= 2.0*np.pi
	# while psi < -np.pi:
	# 	psi += 2.0*np.pi
	
	# erro = (-1)*(psi / np.abs(psi))*erro
    ##############################################################################
    ##### ISSO PRECISA SER RESOLVIDO ( QUAL O SINAL DO ERRO)
	# if psi > 0:
	#  	erro = -erro # necessário inverter sinal pois o erro foi calculado de forma escalar
    
    ###################################################################
	#######################################################################
	# if (ponto_atual[1] < (car.p[0]*(np.tan(car.th) + 1/np.tan (np.pi/2-car.th))-car.p[1])):
	# 	erro = -erro
	 

	#Controle Stanley
	acao_guinada = psi + np.arctan2(( k * erro ), car.v + min)
	if np.abs(acao_guinada) < delta_max: 
		delta = acao_guinada 		
	else:
		if acao_guinada >= delta_max:#saturou positivamente
			delta = delta_max
		else: # saturou negativamente acao_guinada <= -delta_max:
			delta = -delta_max

    
	#print(ponto_atual, erro ,psi)
	#print(psi, erro , pontos_trajetoria[idx])
	#print(pontos_trajetoria[idx], idx, erro, psi)
	print(idx, erro, psi, ptheta[idx] ,  car.th)
	# atua
	car.setSteer(delta)
	
	car.setVel(0.8) # Controle de velocidade basico 
	##########################################################################################

    ###################################################################
   #Caso erro menor que o determinado, passa para próximo ponto	
	# if (np.abs(erro) < 0.1 and np.abs(psi) < 0.5 ):
	# 	index = (index + 1) % np.size(pontos_trajetoria)
	# 	ponto_atual = pontos_trajetoria[index]
	# 	erro = dist_cart(ponto_atual[0], ponto_atual[1], car.p[0], car.p[1])

    ###################################################################
    # lê e exibe camera
	
	img = cv2.flip(cv2.cvtColor(image, cv2.COLOR_BGR2RGB), 0)
	cv2.imshow("Camera",img)
	if cv2.waitKey(1) & 0xFF == ord('q'):
		break
	
	
# plota
plt.clf()
# plota mapa
#plt.imshow(mapa, extent=[-7.5, 7.5, -7.5, 7.5], alpha=0.4)
	
# trajetoria do robo
x = [traj['p'][0] for traj in car.traj]
y = [traj['p'][1] for traj in car.traj]
plt.plot(x, y, 'r')
plt.plot(car.p[0], car.p[1], 'r*')

plt.plot (px, py , 'b')
	
plt.axis('equal')
plt.box(False)
plt.ylabel('y [m]')
plt.ylabel('x [m]')
	
plt.show()
plt.pause(0.01)	

print('Terminou...')
