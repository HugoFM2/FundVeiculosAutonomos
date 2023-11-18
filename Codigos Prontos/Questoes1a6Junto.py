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
vs =  []
dterro = []
dtpsi = []
circulo = True
ligado = 1

#Calcula distância entre dois pontos cartesianos
def dist_cart (x0, y0, xf, yf):
	return np.sqrt((xf - x0)**2 + (yf - y0)**2)


num_pontos = 100
num_pontos_reta = 50

#Traçar pontos da trajetória em 8:
def trajetoria_8 (x0, y0, raio):

	vetores = []
	circulo = False
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
	for i in range (num_pontos_reta):
		x = x0 - raio + i*(raio/num_pontos_reta)
		y = y0
		theta = 0.0
		vetores.append((x, y, theta))
	
    #Trecho reto da origem até o círculo à direita
	for j in range (num_pontos_reta):
		x = x0 + j*(raio/num_pontos_reta)
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
    
	return vetores , circulo

#Traçar pontos da trajetória circular:

def trajetoria_circular_antihorario (x0, y0, raio):
	
	vetores = []
	circulo = True

	angulos = np.linspace(0, 2*np.pi, num_pontos)

	for angulo in angulos:
		x = raio * np.cos(angulo) - raio + x0
		y = raio * np.sin(angulo)  + raio + y0
		
		theta = angulo + np.pi/2	#sentido anti-horário
		theta = theta % (2 * np.pi)
		
		vetores.append((x, y, theta))
		
	    
	return vetores , circulo

#Traçar pontos da trajetória circular:
def trajetoria_circular_horario (x0, y0, raio):
	
	vetores = []
	circulo = True

	angulos = np.linspace(2*np.pi, 0, num_pontos)

	for angulo in angulos:
		x = raio * np.cos(angulo) - raio + x0
		y = raio * np.sin(angulo)  + raio + y0
		
		theta = angulo + 3* np.pi/2	#sentido anti-horário
		theta = theta % (2 * np.pi)
		
		vetores.append((x, y, theta))
		
	    
	return vetores, circulo

#Traçar pontos da trajetória reta:
def trajetoria_reta (x0,y0):
	vetores = []
	circulo = True

	for i in range (num_pontos_reta):
		x = x0 + i*(0.5)
		y = y0 + x
		theta = np.pi/4
		vetores.append((x, y, theta))
	return vetores , circulo

# Função para obter a escolha do usuário
def obter_escolha():
	print("Escolha um tipo de trajetória:")
	print("1. Trajetória em 8")
	print("2. Circular anti-horário")
	print("3. Circular horário")
	print("4. Reta")
    
	escolha = int(input("Digite o número da função desejada (1/2/3/4): "))
	if escolha not in [1, 2, 3, 4]:
		print("Escolha inválida. Tente novamente.")
		return obter_escolha()
		
	return escolha

def switchPID():
	ligado = int(input(" Deseja manter controle longitudinal ligado (1.sim/2.não)"))
	if ligado not in [1, 2]:
		print("Escolha inválida. Tente novamente.")
		return switchPID()
		
	return ligado




#Parâmetros do carro
L = 0.30 # coprimento do carro em metros
delta_max = np.deg2rad(20.0) #máximo de guinada

#Parâmetros do controlador Stanley
k =  0.5# ganho
delta = 0.0 # inicial
min = 0.01

#Parâmetros do contorlador

Kp = 0.8
Ki = 0.05
Kd = 0.04

Vref = 0.5

integral_e = 0.0
derivada_e = 0.0
erro_ant = 0.0
acao_torque = 0.0
delta_t = 0.1
###################################################
# Construção dos vetores de parâmetros da trajetoria

# x0 = -2.0
# y0 = -2.0
# r = 2.0
# pontos_trajetoria = trajetoria_8 (x0 , y0 , r)
#pontos_trajetoria = trajetoria_circular_antihorario (x0 , y0 , r)
#pontos_trajetoria = trajetoria_circular_horario (x0 , y0 , r)
#pontos_trajetoria = trajetoria_reta(x0,y0)

escolha = obter_escolha()
x0 = float(input("Digite o x do centro da trajetória: "))
y0 = float(input("Digite o y do centro da trajetória: "))
if escolha != 4:
 	r = float(input("Digite o raio: "))
    
if escolha == 1:
    pontos_trajetoria , circulo = trajetoria_8 (x0 , y0 , r)
elif escolha == 2:
   	pontos_trajetoria, circulo = trajetoria_circular_antihorario (x0 , y0 , r)
elif escolha == 3:
	pontos_trajetoria,circulo = trajetoria_circular_horario (x0 , y0 , r)	   
else:
	pontos_trajetoria,circulo = trajetoria_reta (x0 , y0)


print ("Há um bug no simulador que causa instabilidade no Controle Longitudinal")
ligado = switchPID ()
print("Veja a trajetória gerada na janela que se abriu. Feche-a para iniciar a simulação")



px = []
py = []

idx = 0
px = [i[0] for i in pontos_trajetoria]
py = [i[1] for i in pontos_trajetoria]
ptheta = [i[2] for i in pontos_trajetoria]


plt.figure()
plt.plot (px, py)
plt.show()
distancias_curva = []

primeiro = (True)

# mapa de chao
mapa = cv2.cvtColor(cv2.imread('./coppelia/pista.png'), cv2.COLOR_RGB2BGR)



#iNÍCIO DO LOOP
while car.t < 50.0:
	
	# lê sesnores
	car.step()

	# Pega a camera
	image = car.getImage()
	

	#Distância do carro a todos os posntos da curva
	distancias_curva = []
	for i in range (len(pontos_trajetoria)): 
		aux = dist_cart(px[i], py[i], car.p[0], car.p[1])
		distancias_curva.append(aux)
	idx = np.argmin(distancias_curva)
	
	if primeiro:
		if idx == 0:
			idx = 1
		idx_ant = idx - 1 
		theta_anterior = car.th #para um filtro de car.th
		primeiro = False
	
	i=0
	while (idx < idx_ant or idx > idx_ant+10):
		idx = np.argpartition(distancias_curva, i)[i]
		i+=1
		if i>=len(distancias_curva):
			break
	idx_ant = idx
	
	if circulo:
		if idx > (0.95*(num_pontos)): #Completou mais que 95% do percurso reinicia trajetoria
			idx = 1
			idx_ant =0
	else:
		if idx > (0.95*(2*num_pontos+4*num_pontos_reta)): #Completou mais que 95% do percurso
			idx = 1
			idx_ant =0
	
	# if idx > (0.95*(num_pontos)): #Completou mais que 95% do percurso reinicia trajetoria
	# 	idx = 1
	# 	idx_ant =0
	# if idx > (0.95*(2*num_pontos+4*num_pontos_reta)): #Completou mais que 95% do percurso
	# 	idx = 1
	# 	idx_ant =0
	#################################################################################################
    # cálculo do erro longitudinal

	erro_vel = (Vref-car.v)
	integral_e += erro_vel*delta_t
	derivada_e = (erro_vel - erro_ant)/delta_t

	erro_ant = erro_vel
	#####################################################
    #controle PID Longitudinal
	####################################################
	acao_torque = Kp * erro_vel + Ki * integral_e + Kd * derivada_e

	# atua
	if ligado == 1:
		car.setU (acao_torque)
		#print('ligado')
	else:
		car.setU (0.05)
		#print('delisgado')


	
	#################################
	## Cálculo erro lateral
	################################
	vetor_lateral_carro = [	-np.cos(car.th + np.pi / 2),
                      		-np.sin(car.th + np.pi / 2)]
	
	vetor_curv_carro = [px[idx]-car.p[0],
					 	py[idx]-car.p[1]]
	
	erro = np.dot(vetor_curv_carro, vetor_lateral_carro)
    
	#############################
	#Calculo de PSI
	##########################

	###Código funcionando

	# if  np.abs (car.th - ptheta[idx]) > np.pi: ## tratamento erro numerico - quando car.th 1Q e orientacao 4Q
	# 	psi = (car.th -ptheta[idx]) - 2*np.pi
		
	# else:	
	# 	psi = (car.th - ptheta[idx])
	# ############################################################################	

	##### Funciona também
	# if  np.abs (car.th - ptheta[idx]) > np.pi: ## tratamento erro numerico - quando car.th 1Q e orientacao 4Q
	# 	psi = (car.th -ptheta[idx])
	# 	if (car.th < ptheta[idx]):
	# 		psi = psi + 2*np.pi
	# 	else:
	# 		psi = psi - 2*np.pi		
	# else:	
	# 	psi = (car.th - ptheta[idx])


	################################################
	#Uma tentativa filtro porque car.th oscila bruscamente quando há uma inclinação, mesmo que pequena do eixo Z
	
	if  np.abs(theta_anterior - car.th) > np.pi: ## tratamento erro numerico - quando car.th 1Q e orientacao 4Q
		aux = (theta_anterior - car.th) - 2*np.pi
		
	else:	
		aux = (theta_anterior - car.th)

	if aux > np.pi/2:
		ctheta = theta_anterior
	else:
		ctheta = car.th

	theta_anterior = ctheta

	if np.abs(theta_anterior - car.th) > np.pi/2:
		ctheta = theta_anterior
	else:
		ctheta = car.th

	# theta_anterior = ctheta

	psi = (ctheta - ptheta[idx])
	
	#psi = (car.th - ptheta[idx])
	while psi > np.pi:
		psi -= 2.0*np.pi
	while psi < -np.pi:
		psi += 2.0*np.pi
	
    ##############################################################################
   	#Controle Stanley
	acao_guinada = psi + np.arctan2(( k * erro ), car.v + min)
	if np.abs(acao_guinada) < delta_max: 
		delta = acao_guinada 		
	else:
		if acao_guinada >= delta_max:#saturou positivamente
			delta = delta_max
		else: # saturou negativamente acao_guinada <= -delta_max:
			delta = -delta_max
  

	print(idx, erro, psi, ptheta[idx] , car.th)

	# atua
	car.setSteer(delta)	
	#car.setVel(Vref) # Controle de velocidade basico 
	
	vs.append(car.v)
	time.append(round(car.t,2))
	x.append(car.p[0])
	y.append(car.p[1])
	dterro.append (erro)
	dtpsi.append(psi)


    # lê e exibe camera
	
	img = cv2.flip(cv2.cvtColor(image, cv2.COLOR_BGR2RGB), 0)
	cv2.imshow("Camera",img)
	if cv2.waitKey(1) & 0xFF == ord('q'):
		break

# Salva os dados em um formato csv
import pandas as pd
df = pd.DataFrame({'tempo'    : time,
					'vs'   : vs,
					'x'		:x,
					'y'		:y,
					'erro'	:dterro,
					'psi'	:dtpsi,
					'vref'	:Vref*np.ones(len(time))})
df.to_csv('Dados_StanleyCH.csv', index=True)	
	
# plota
plt.clf()
#plota mapa
plt.imshow(mapa, extent=[-7.5, 7.5, -7.5, 7.5], alpha=0.4)
	
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