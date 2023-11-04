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

# mapa de plot
mapa = cv2.cvtColor(cv2.imread('../coppelia/pista.png'), cv2.COLOR_RGB2BGR)

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

WHITE_HSV_MIN =  [0,0,150] # PISTA CIRCUITO
WHITE_HSV_MAX = [255,40,255] # PISTA CIRCUITO

# Inicializacao do theta e rho
theta = 0
rho = 300



def blob(image,min_hsv,max_hsv,color=(0,255,0)):
	# Detecta blobs em HSV e retorna sua mascara
	hsv = cv2.cvtColor(image,cv2.COLOR_BGR2HSV) # Convert to hsv

	lower_color = np.array(min_hsv)
	upper_color = np.array(max_hsv)

	mask = cv2.inRange (hsv,lower_color,upper_color)

	return mask


def updateGraph(fig,grafico,x,y):
	# # update data
	grafico.set_data(x,y)	
	canvas = FigureCanvas(fig)
	canvas.draw()

	graph_image = np.array(fig.canvas.get_renderer()._renderer)
	return graph_image


def getStreetMarks(image):
	# Aplica uma dilatacao nas linhas brancas e verifica se todo o seu contorn esta na pista, caso nao esteja,
	# é uma linha de lado e nao deve ser detectada

	image_test = image.copy()
	hsv = cv2.cvtColor(image_test,cv2.COLOR_BGR2HSV) # Convert to hsv

	# Detecta a Area Branca (sua mascara)
	mask_white = blob(image_test,WHITE_HSV_MIN,WHITE_HSV_MAX, color=(255,255,255))
	# cv2.imshow("BLOB", mask_white)

	# Expande a area 
	kernel_size = 8
	kernel  = np.ones((kernel_size,kernel_size), np.uint8)
	dilated_mask = cv2.dilate(mask_white, kernel, iterations=2)	

	# Encontra os contornos da area expandida
	contours,_ = cv2.findContours(dilated_mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)


	selected_contours = []
	unselected_contours = []
	for contour in contours:
		mask = np.zeros(hsv.shape, np.uint8)

		mask = cv2.cvtColor(mask, cv2.COLOR_BGR2GRAY)

		# Aplica o contorno da imagem expandida
		cv2.drawContours(mask, [contour], -1, 255, 1) 
		mean = cv2.mean(hsv, mask=mask)
		area = cv2.contourArea(contour) # Calculate area

		if  (mean[1] < 15): # Baixo valor de saturation
			selected_contours.append(contour)

		

	cv2.drawContours(hsv, selected_contours,-1,(255,255,0),2) # Green color around

	
	# cv2.imshow("HSV", hsv) # DEBUG

	mask_lines = np.zeros(hsv.shape, np.uint8)


	cv2.drawContours(mask_lines, selected_contours,-1,(255,255,255),-1) # Green color around
	# cv2.imshow("Sem Canto", mask_lines) # DEBUG

	return mask_lines,selected_contours





def getLine(imageStreetMark,selected_contours):
	# Obtem uma imagem em preto contendo apenas a linha desejada( mais proximo do carrinho)
	lines_mask = cv2.cvtColor(imageStreetMark.copy(), cv2.COLOR_BGR2GRAY)

	mask_lines = np.zeros(lines_mask.shape, np.uint8)

	
	kernel_size = 4
	kernel  = np.ones((kernel_size,kernel_size), np.uint8)
	eroded_image = cv2.erode(lines_mask, kernel, iterations=5)

	edges = cv2.Canny(eroded_image, 50, 155)


	## ABORDAGEM 4 -> STACKOVERFLOW SKELETON https://stackoverflow.com/questions/58247563/middle-line-between-2-contour-lines-in-opencv-python
	img = cv2.cvtColor(imageStreetMark.copy(), cv2.COLOR_BGR2GRAY)
	# do some eroding of img, but not too much
	kernel = np.ones((5,5), np.uint8)
	img = cv2.erode(img, kernel, iterations=2)

	# threshold img
	ret, thresh = cv2.threshold(img,127,255,0)

	# do distance transform
	dist = cv2.distanceTransform(thresh, distanceType=cv2.DIST_L2, maskSize=5)

	# set up cross for tophat skeletonization
	kernel = cv2.getStructuringElement(cv2.MORPH_CROSS,(3,3))
	skeleton = cv2.morphologyEx(dist, cv2.MORPH_TOPHAT, kernel)

	# threshold skeleton
	ret, skeleton = cv2.threshold(skeleton,0,255,0)
	# cv2.imshow("Skeleton", skeleton)
	skeleton = np.uint8(skeleton)
	edges = cv2.Canny(skeleton, 50, 155)
	# Obtem TODAS as linhas
	linesP = cv2.HoughLinesP(edges,rho=1,theta=0.1*np.pi/180,threshold=20,minLineLength=10,maxLineGap=200)
	lines = []
	if linesP is not None:
		for i in range(0, len(linesP)):
			l = linesP[i][0]
			lines.append(l)


		# Obtem apenas a linha mais proxima
		# Ordena os pontos de acordo com o ponto central inferior da imagem
		x_middle = img.shape[1]/2
		# sorted_lines = sorted(lines, key=lambda line: min(math.dist((line[0], line[1]), (x_middle, 0)),
        #                                           math.dist((line[2], line[3]), (x_middle, 0))))
        # ORdena a partir dos que tem o menor y
		sorted_lines = sorted(lines, key=lambda line: min( abs(line[1]-0),
                                                  abs(line[3]-0) ))

		# Obtem a linha mais proxima do centro inferior da imagem, ou seja, a primeira linha vista pelo carrinho
		closest_line = sorted_lines[0]

		# Caso x0 e x1 estejam invertidos
		x0, y0, x1, y1 = closest_line
		if abs(x0 - 300) > abs(x1 - 300):
		    closest_line = (x1, y1, x0, y0)

		# Desenha a linha mais proxima
		cv2.line(mask_lines, (closest_line[0], closest_line[1]), (closest_line[2], closest_line[3]), (255,255,255), 3, cv2.LINE_AA)


	# Retorna uma imagem contendo apenas a linha mais proxima detectada
	return mask_lines



def getInclination(imageMask,showImage=True):
	# Obtem a inclinacao e distancia da linha mais proxima
	global theta,rho
	mask = np.zeros(imageMask.shape, np.uint8)
	# print("MASK SHAPE:",mask.shape)
	# calcula Hough Line Transform
	edges = cv2.Canny(imageMask, 50, 155)
	cv2.imshow("GETINCLINATION", edges) # DEBUG

	lines = cv2.HoughLines(edges, 1, np.pi/180, threshold=60, min_theta=np.radians(-100), max_theta=np.radians(100))  # Adjust threshold as needed

	# desenha linhas e calcula a media delas
	x1=300
	linhas = []
	if lines is not None:
		for rho, theta in lines[:, 0]:
			if theta > np.pi/2:
				continue
			linhas.append( (theta,rho) )
		
			# Faz a media de theta e rho obtidos
			theta = sum([x[0] for x in linhas])/len(linhas)
			rho = sum([x[1] for x in linhas])/len(linhas)
			# print(f"DETECTOU: {theta}")

	#Theta corresponde ao psi e o rho corresponde ao x
	a = np.cos(theta)
	b = np.sin(theta)
	x0 = a * rho
	y0 = b * rho
	x1 = int(x0 + 1000 * (-b))
	y1 = int(y0 + 1000 * (a))
	x2 = int(x0 - 1000 * (-b))
	y2 = int(y0 - 1000 * (a))
	print(f"x1:{x1}/y1:{y1}/x2:{x2}/y2:{y2}")
	cv2.line(mask, (x1, y1), (x2, y2), (255, 255, 255), 2)  # Desenha a curva do carro
	m = (y2 - y1) / (x2 - x1)
	b_constant = y1 - m * x1

	x_line = (mask.shape[0] - b_constant) / m # Ponto em que y = 300

	# Desenha a linha central do modelo (Apenas para debug) # ATENCAO, O EIXO 0,0 É O CANTO SUPERIOR ESQUERDO DA IMAGEM
	x1_car = int(mask.shape[1]/2) # 300
	y1_car = int(mask.shape[0]) # 600, a parte inferior
	x2_car = int(mask.shape[1]/2) #300
	y2_car = int(0) # 0 (O topo)
	cv2.line(mask, (x1_car, y1_car), (x2_car, y2_car), (255, 255, 255), 2)  # Draw lines in red


	if showImage:
			cv2.imshow("Linhas", mask)

	print(f'y0:{y0} / rho: {rho} / a: {a} / b:{b} / X_LINE:{x_line}')

	return theta,x_line


def ControleLateral(car_x,line_x,line_psi,car_psi):
	# k = 1.2 #ganho
	k = 0.2 #ganho COM VEL 0.3
	# k=1.2
	ke = 0.00392 # Multiplicador do erro, transforma pixels para m
	delta_max = np.deg2rad(20.0) #máximo de guinada

	# cálculo do erro e psi no caso da reta
	erro = (line_x-car_x) # menos a posição x do carro
	psierro = -(line_psi)
	print(f'line_x:{line_x} / erro:{round(erro,5)} / psierro: {round(psierro,2)} /2o erro:{np.arctan2( k * erro , car.v)}')

	#Controle Stanley
	acao_guinada = psierro + np.arctan2( k * ke * erro , car.v)
	if np.abs(acao_guinada) < delta_max: 
	    delta = acao_guinada 		
	else:
		if acao_guinada >= delta_max:#saturou positivamente
			delta = delta_max
		else: # saturou negativamente acao_guinada <= -delta_max:
			delta = -delta_max

	return delta,erro,psierro





fig = plt.figure()
plt.imshow(mapa, extent=[-7.5, 7.5, -7.5, 7.5], alpha=0.99)
grafico, = plt.plot(x,y)



theta_anterior = 0 #para um filtro de car.th
# image = cv2.imread('../../../Imagens Teste/Erro_Baixo.png') # IMAGEM FIXA
while car.t < 100:
	
	# lê sensores
	car.step()

	# Pega a camera
	image = car.getImage()

	image = cv2.flip(cv2.cvtColor(image, cv2.COLOR_BGR2RGB), 0) # Inverte a imagem


	showImage= True
	################################
	# CALCULO DA VISAO COMPUTACIONAL
	################################
	# Detecta as linhas e tenta seguir elas de acordo com a angulacao

	#Obtem altura(h->y) e largura(w->x) da imagem
	h,w,_ = image.shape

	# Corta a parte superior da imagem, para obter apenas a area desejada
	top = int(h*0.5)
	image_without_top = image[1-top:h, 0:w]

	# Obtem as linhas centrais
	imageStreetMark,selected_contours = getStreetMarks(image_without_top)

	# Obtem a linha mais proxima do carro
	mask_lines = getLine(imageStreetMark,selected_contours)

	# Calcula a inclinacao e distancia do centro da linha mais proxima do carro
	theta,rho = getInclination(mask_lines,showImage=showImage)


	################################
	# FIM VISAO COMPUTACIONAL
	################################

	
	# lê e exibe camera
	if showImage:
		cv2.imshow("Camera",image)


	# Faz o controle lateral
	delta,erro,psierro = ControleLateral(car_x=300,line_x=rho,line_psi=theta,car_psi=0)
	car.setSteer(delta) 

	# Controle Longitudinal
	# car.setVel(0.3)
	car.setVel(0.4)
	# car.setVel(0.3)
	# car.setVel(0.6)
	# car.setU(0)
	# car.setSteer(0)


	# # Exibe o grafico da posicao do carrinho no mapa -> FUNCAO LENTA
	# img_grafico = updateGraph(fig,grafico,x,y)
	# cv2.imshow("Grafico",img_grafico)


	# Armazena valores de posicao e tempo do carrinho
	x.append(car.p[0])
	y.append(car.p[1])
	time.append(car.t)
	erros.append(erro)
	psierros.append(psierro)
	vs.append(car.v)
	steers.append(delta)

	print(car.t)


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
df.to_csv('Q10.csv', index=True)
