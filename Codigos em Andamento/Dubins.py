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
import DubinsCurve as dp
import numpy as np
import cv2,math
import matplotlib.pyplot as plt
plt.ion()
plt.figure(1)


### DEFININDO PONTOS DO INFINITO E ORIENTACAO
amplitude = 8
frequency = 0.5  
num_points = 200 


t = np.linspace(0, 4 * np.pi, num_points)

scale = 10 / (2-np.cos(2*t))

x = scale * np.cos(t)
y = scale * np.sin(2*t)


xs = scale * np.cos(t)
ys = scale * np.sin(2*t)/2

# Calcula Orientacao
slopes = []
for i in range(len(xs) -1):
	vector_x = xs[i+1] - xs[i]
	vector_y = ys[i+1] - ys[i]
	theta = math.atan2(vector_y,vector_x)
	slopes.append(theta)


# R = L / tan(delta)
RHO_MIN = 1.2 * (0.3/np.tan(cp.CAR['STEERMAX']))

# mapa de chao
mapa = cv2.cvtColor(cv2.imread('../coppelia/pista.png'), cv2.COLOR_RGB2BGR)

########################################
def controleLateral(car, curve):
	
	# pega os pontos da curva
	points = curve.getSamplingPoints()
	px = points[:, 0]
	py = points[:, 1]
		
	# ponto na curva com a menor distancia
	dist = [np.linalg.norm(car.p - np.array([px[i], py[i]])) for i in range(len(px))]
	idx = np.argmin(dist)
	
	# referencia de controle
	try:
		xc, yc = car.p
		X1, Y1 = px[idx], py[idx]
		X2, Y2 = px[idx+1], py[idx+1]
		PSIREF = np.arctan2(Y2-Y1, X2-X1)
	except:
		return 0.0, True
	
	num = (X2-X1) * (Y1-yc) - (X1-xc) * (Y2-Y1)
	den = np.sqrt((X2-X1)**2.0 + (Y2-Y1)**2.0)
	e = num/den
	
	# psie entre [-pi..pi]
	psie = PSIREF - car.th
	while psie > np.pi:
		psie -= 2.0*np.pi
	while psie < -np.pi:
		psie += 2.0*np.pi
	
	# ganhos
	# K = 10.0
	# Kd = 5.0
	# ks = 0.1
	K = 0.8
	Kd = 0
	ks = 0
	
	# lei de controle
	delta = psie + np.arctan2(K*e, car.v + ks) - Kd*(0 - car.w)
	
	return -delta, False

########################################
# cria comunicação com o carrinho
car = cp. CarCoppelia()
car.startMission()

# cria a curva de Dubins




while car.t < 50.0:
	

	

	for i in range(len(xs)):
		qi = np.array([car.p[0], car.p[1], car.th])
		qf = np.array([xs[i], ys[i], slopes[i]])
		curve = dp.DubinsCurve(qi, qf, rhomin=1*RHO_MIN)
		while True:
			# lê senores
			car.step()

			# seta direcao
			delta, reach = controleLateral(car, curve)
			if reach:
				break
			
			car.setSteer(delta)


	
			# atua
			car.setVel(1.0)
			
			# ------------------
			# plota
			plt.clf()
			# plota mapa
			plt.imshow(mapa, extent=[-7.5, 7.5, -7.5, 7.5], alpha=0.4)
			
			# curva de Dubins
			curve.draw(plt.gca(), drawHeadings=True)
			
			# trajetoria do robo
			x = [traj['p'][0] for traj in car.traj]
			y = [traj['p'][1] for traj in car.traj]
			plt.plot(x, y, 'r')
			plt.plot(car.p[0], car.p[1], 'r*')
			
			plt.axis('equal')
			plt.box(False)
			plt.ylabel('y [m]')
			plt.ylabel('x [m]')
			
			plt.show()
			plt.pause(0.01)
	
plt.pause(10.0)
	
print('Terminou...')