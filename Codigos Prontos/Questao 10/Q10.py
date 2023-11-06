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

WHITE_HSV_MIN =  [0,0,150] # PISTA CIRCUITO
WHITE_HSV_MAX = [255,40,255] # PISTA CIRCUITO

# Inicializacao do theta e rho
angulo = 0
x_line = 300

def mouseRGB(event,x,y,flags,param): # AUXILIAR
	if event == cv2.EVENT_LBUTTONDOWN: #checks mouse left button down condition
		colorsB = image[y,x,0]
		colorsG = image[y,x,1]
		colorsR = image[y,x,2]
		colors = image[y,x]
		print("Red: ",colorsR)
		print("Green: ",colorsG)
		print("Blue: ",colorsB)
		print("BRG Format: ",colors)
		print("Coordinates of pixel: X: ",x,"Y: ",y)

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


def distance_to_line(x1, y1, x2, y2, x0, y0):
	# Calculate the perpendicular distance from point (x0, y0) to the line defined by (x1, y1) and (x2, y2)
	return np.abs((y2 - y1) * x0 - (x2 - x1) * y0 + x2 * y1 - y2 * x1) / np.sqrt((y2 - y1)**2 + (x2 - x1)**2)


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

		# ORdena a partir dos que tem o menor y
		min_distance = 3000
		point = (0,img.shape[1]/2)
		for line in lines:
			x1, y1, x2, y2 = line
			distance = distance_to_line(x1, y1, x2, y2, point[0], point[1])
			if distance < min_distance:
				min_distance = distance
				closest_line = line


		# Caso x0 e x1 estejam invertidos
		x0, y0, x1, y1 = closest_line
		if abs(x0 - 300) > abs(x1 - 300):
			closest_line = (x1, y1, x0, y0)

		# Desenha a linha mais proxima
		cv2.line(mask_lines, (closest_line[0], closest_line[1]), (closest_line[2], closest_line[3]), (255,255,255), 3, cv2.LINE_AA)

		# cv2.imshow('mask_lines',mask_lines) # DEBUG

	# Retorna uma imagem contendo apenas a linha mais proxima detectada
	return mask_lines



def getInclination(imageMask,showImage=True):
	# Obtem a inclinacao e distancia da linha mais proxima
	global angulo,x_line
	mask = np.zeros(imageMask.shape, np.uint8)
	# print("MASK SHAPE:",mask.shape)
	# calcula Hough Line Transform
	edges = cv2.Canny(imageMask, 50, 155)
	cv2.imshow("GETINCLINATION", edges) # DEBUG

	lines = cv2.HoughLines(edges, 1, np.pi/180, threshold=60, min_theta=np.radians(-100), max_theta=np.radians(100))  # Adjust threshold as needed

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
		

		#Tratamento em que os pontos se invertem
		if x1 > x2:
			# Swap the points
			x1, x2 = x2, x1
			y1, y2 = y2, y1

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



		# Desenha as linhas para visualicacao do tracejado e da orientacao do carro
		cv2.line(image_without_top, (x1, y1), (x2, y2), (255, 0, 0), 2)  # Desenha a linha detectada
		cv2.line(image_without_top, (x1_car, y1_car), (x2_car, y2_car), (0, 255, 0), 2)  # Desenha linha do carro
		cv2.line(image_without_top, (int(x_line), int(mask.shape[0])), (x2_car, y2_car), (0, 255, 255), 2)  # Desenha linha do carro

		# cv2.imshow("Linhas", mask) # DEBUG
		angulo = math.atan(m)
		print("angulo:",angulo - np.pi/2)

	return -angulo,x_line


def ControleLateralVisao(car_x,line_x,line_psi,car_psi,k=1.0):
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


	#Controle Stanley
	vel_car = car.v
	if vel_car == 0:
		vel_car = 0.00001
	acao_guinada = psierro + np.arctan(( k * erro) / vel_car)
	if np.abs(acao_guinada) < delta_max: 
		delta = acao_guinada 		
	else:
		if acao_guinada >= delta_max:#saturou positivamente
			delta = delta_max
		else: # saturou negativamente acao_guinada <= -delta_max:
			delta = -delta_max

	return delta,erro,psierro





# image_without_top = cv2.imread('../../../Imagens Teste/ROAD7_MISSDETECT.png') # IMAGEM FIXA
while car.t < 113:
	
	# lê sensores
	car.step()

	# Pega a camera
	image = car.getImage()

	#Obtem altura(h->y) e largura(w->x) da imagem
	h,w,_ = image.shape

	image = cv2.flip(cv2.cvtColor(image, cv2.COLOR_BGR2RGB), 0) # Inverte a imagem

	# Adiciona uma linha cinza embaixo para detectar as linhas que estao em contato com a parte de baixo
	blank_image = np.zeros((h+10,w,3), np.uint8)
	blank_image[:,:] = (47,47,47)

	l_img = blank_image.copy()                    # (600, 900, 3)

	x_offset = 0
	y_offset = 5
	# Here, y_offset+height <= blank_image.shape[0] and x_offset+width <= blank_image.shape[1]
	l_img[y_offset:y_offset+h, x_offset:x_offset+w] = image.copy()
	image = l_img
	h,w,_ = image.shape

	showImage= True
	################################
	# CALCULO DA VISAO COMPUTACIONAL
	################################
	# Detecta as linhas e tenta seguir elas de acordo com a angulacao



	# Corta a parte superior da imagem, para obter apenas a area desejada
	# top = int(h*0.5)
	top = int(h*0.7)
	# top = int(h*0.5)
	image_without_top = image[1-top:h, 0:w]
	cv2.imshow("image_without_top",image_without_top)
	h,w,_ = image_without_top.shape


	


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
		cv2.namedWindow('Camera')
		cv2.setMouseCallback('Camera',mouseRGB)
		cv2.imshow("Camera",image)


	# Faz o controle lateral
	# delta,erro,psierro = ControleLateralVisao(car_x=300,line_x=rho,line_psi=theta,car_psi=0,k=0.9)
	delta,erro,psierro = ControleLateralVisao(car_x=300,line_x=rho,line_psi=theta,car_psi=0,k=1.6)


	car.setSteer(delta) 

	# Controle Longitudinal
	# car.setVel(0.4)
	car.setU(0.05)

	print("VELOCIDADE:",car.v)

	# # Exibe o grafico da posicao do carrinho no mapa ao vivo -> FUNCAO LENTA
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

# Exibe o grafico da posicao
# mapa de plot
mapa = cv2.cvtColor(cv2.imread('../../coppelia/pista.png'), cv2.COLOR_RGB2BGR)
fig = plt.figure()
plt.imshow(mapa, extent=[-7.5, 7.5, -7.5, 7.5], alpha=0.99)
grafico, = plt.plot(x,y,'y')
plt.show()


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
