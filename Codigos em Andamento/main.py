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

GREEN_HSV_MIN = [35,150,50]
GREEN_HSV_MAX = [85,255,255]
BLUE_HSV_MIN =  [90,150,50]
BLUE_HSV_MAX = [130,255,255]

WHITE_HSV_MIN =  [0,0,150] # PISTA CIRCUITO
WHITE_HSV_MAX = [255,40,255] # PISTA CIRCUITO

# WHITE_HSV_MIN =  [0,0,180]
# WHITE_HSV_MAX = [2,10,210]

def roadDetection(frame_bgr):
	
	# converte pra tons de cinza
	frame_gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
	
	# aplica filtro
	frame_blur = cv2.GaussianBlur(frame_gray, (5,5), 0)
	
	# detecta bordas
	edges = cv2.Canny(frame_blur, 50, 155)
	cv2.imshow("Edges",edges)	
	# # calcula Hough Line Transform
	# lines = cv2.HoughLines(edges, 1, np.pi/180, threshold=30, min_theta=np.radians(-60), max_theta=np.radians(60))  # Adjust threshold as needed

	# # desenha linhas
	image_with_lines = frame_bgr.copy()
	
	# if lines is not None:
	# 	for rho, theta in lines[:, 0]:
			
	# 		if theta > np.pi/2:
	# 			continue
			
	# 		a = np.cos(theta)
	# 		b = np.sin(theta)
	# 		x0 = a * rho
	# 		y0 = b * rho
	# 		x1 = int(x0 + 1000 * (-b))
	# 		y1 = int(y0 + 1000 * (a))
	# 		x2 = int(x0 - 1000 * (-b))
	# 		y2 = int(y0 - 1000 * (a))
	# 		c_x = (x1+x2)/2
	# 		cv2.line(image_with_lines, (x1, y1), (x2, y2), (255, 0, 0), 2)  # Draw lines in red

	# Calcula o Hough Lines P
	# linesP = cv2.HoughLinesP(edges, lines=1, rho=np.pi / 180, theta=50, threshold=40, minLineLength=50, maxLineGap=10)
	linesP = cv2.HoughLinesP(edges, 1, np.pi / 180, 50, 20, 50, 1)
	if linesP is not None:
		print(linesP)
		for i in range(0, len(linesP)):
			l = linesP[i][0]
			cv2.line(image_with_lines, (l[0], l[1]), (l[2], l[3]), (0,0,255), 3, cv2.LINE_AA)

			
	return image_with_lines

# Detecta blobs em HSV
def blob(image,min_hsv,max_hsv,color=(0,255,0)):
	hsv = cv2.cvtColor(image,cv2.COLOR_BGR2HSV) # Convert to hsv

	lower_color = np.array(min_hsv)
	upper_color = np.array(max_hsv)

	mask = cv2.inRange (hsv,lower_color,upper_color)

	contours,_ = cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)

	

	largest_area = 0
	largest_contour = None

	for i, contour in  enumerate(contours):
		area = cv2.contourArea(contour) # Calculate area

		# if the curent blob has a larger area, update the largest area and contour
		if area > largest_area:
			largest_area = area
			largest_contour = contour



	cx = cy = -1
	if largest_contour is not None:
		M = cv2.moments(largest_contour)
		cx = int(M['m10'] / M["m00"])
		cy = int(M['m01'] / M["m00"])

		# cv2.drawContours(image, [largest_contour],-1,color,2) # Green color around

	return image, largest_area,cx,cy,mask


def updateGraph(fig,grafico,x,y):
	# # update data
	# grafico.set_ydata(x[-1])
	# grafico.set_ydata(y[-1])

	# # redraw the canvas
	# fig.canvas.draw()

	# # convert canvas to image
	# img = np.fromstring(fig.canvas.tostring_rgb(), dtype=np.uint8,
	#         sep='')
	# img  = img.reshape(fig.canvas.get_width_height()[::-1] + (3,))

	# # img is rgb, convert to opencv's default bgr
	# img = cv2.cvtColor(img,cv2.COLOR_RGB2BGR)

	grafico.set_data(x,y)
	# plt.plot(x,y,'r')
	# return img	
	canvas = FigureCanvas(fig)
	canvas.draw()

	graph_image = np.array(fig.canvas.get_renderer()._renderer)
	return graph_image

def dilateImage(image,mask,kernel_size,iterations=1):
	# Dilate MASK
	kernel  = np.ones((kernel_size,kernel_size), np.uint8)
	dilated_mask = cv2.dilate(mask, kernel, iterations=iterations)	

	# Find contour from dilated mask
	# contours,_ = cv2.findContours(dilated,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)

	# cv2.cvtColor(dilated_mask,cv2.COLOR_GRAY2RGB)
	# not_mask = cv2.bitwise_not(dilated_mask)
	pixel_cinza = [46,46,46]
	# background = np.full(image.shape, pixel_cinza, dtype=np.uint8)
	# bk = cv2.bitwise_or(background, background, mask=not_mask)
	# cv2.imshow("BK1", bk)
	image_dilated = image.copy()

	image_dilated[dilated_mask > 0] = pixel_cinza
	# image_dilated = cv2.subtract(image, cv2.cvtColor(dilated_mask,cv2.COLOR_GRAY2RGB))
	
	# canvas = cv2.drawContours(image.copy(), contours, -1, (0,255,0), 2)
	# image_without_dilated = cv2.subtract(image, bk)


	# image_without_dilated = cv2.bitwise_or(bk,image)

	cv2.imshow("BK", dilated_mask)

	return image_dilated




def getStreetMarks(image):
	# Aplica uma dilatacao nas linhas brancas e verifica se todo o seu contorn esta na pista, caso nao esteja,
	# é uma linha de lado e nao deve ser detectada

	image_test = image.copy()
	# image_test = cv2.imread('F:/Google Drive/UFMG/2023.2/Veiculos Autonomos/Projeto 1/FVA_simulacao/Imagens Teste/Road1.png')
	hsv = cv2.cvtColor(image_test,cv2.COLOR_BGR2HSV) # Convert to hsv
	# Detecta a Area
	image,white_area,cx_white,cy_white,mask_white = blob(image_test,WHITE_HSV_MIN,WHITE_HSV_MAX, color=(255,255,255))
	# cv2.imshow("BLOB", mask_white)

	# Expande a area 
	# kernel_size = 10 # QUESTAO DO CIRCUITO
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
		# print(area)
		# print(mean)
		# if  mean[0] < 80 and mean[1] < 70 and mean[2] < 70:
		if  (mean[1] < 15): # Baixo valor de saturation
			selected_contours.append(contour)

		

	cv2.drawContours(hsv, selected_contours,-1,(255,255,0),2) # Green color around
	# cv2.drawContours(hsv, unselected_contours,-1,(0,255,0),2) # Green color around
	
	cv2.imshow("HSV", hsv)

	mask_lines = np.zeros(hsv.shape, np.uint8)


	cv2.drawContours(mask_lines, selected_contours,-1,(255,255,255),-1) # Green color around
	cv2.imshow("Sem Canto", mask_lines)

	return mask_lines,selected_contours


def distance(p1, p2):
	return ((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2) ** 0.5

def sort_points(points):
    sorted_points = [points[0]]  # Initialize the sorted list with the first point
    
    while len(sorted_points) < len(points):
        last_point = sorted_points[-1]
        remaining_points = [point for point in points if point not in sorted_points]
        closest_point = min(remaining_points, key=lambda x: distance(last_point, x))
        sorted_points.append(closest_point)
    
    return sorted_points


def getLine(imageStreetMark,selected_contours):
	lines_mask = cv2.cvtColor(imageStreetMark.copy(), cv2.COLOR_BGR2GRAY)
	
	

	mask_lines = np.zeros(lines_mask.shape, np.uint8)

	
	kernel_size = 4
	kernel  = np.ones((kernel_size,kernel_size), np.uint8)
	eroded_image = cv2.erode(lines_mask, kernel, iterations=5)

	edges = cv2.Canny(eroded_image, 50, 155)

	
	# ### ABORDAGEM 0 -> FAZER UM Linha contida em cada contorno
	# rows,cols = lines_mask.shape[:2]
	# if selected_contours is not None:
	# 	for contour in selected_contours:
	# 		[vx,vy,x,y] = cv2.fitLine(contour, cv2.DIST_L2,0,0.01,0.01)
	# 		lefty = int((-x*vy/vx) + y)
	# 		righty = int(((cols-x)*vy/vx)+y)
	# 		# print(righty)
	# 		# righty = (((cols-x)*vy/vx)+y)
	# 		cv2.line(mask_lines,(cols-1,righty),(0,lefty),(255,255,255),2)

	# ### ABORDAGEM 1, DETECTAR OS CENTROS E TENTAR FAZER UMA LINHA DESSES CENTROS
	# # cx = cy = -1
	# centros = [(int(eroded_mask.shape[1]/2),int(eroded_mask.shape[0]))]
	# if selected_contours is not None:
	# 	for contour in selected_contours:
	# 		M = cv2.moments(contour)
	# 		cx = int(M['m10'] / M["m00"])
	# 		cy = int(M['m01'] / M["m00"])
	# 		centros.append((cx,cy))
	# 		cv2.circle(edges, (cx, cy), 4, 255, -1)

	# target = (eroded_mask.shape[0]/2,0) # O ponto que se deseja sortear todos os outros pontos é o ponto centro inferior
	# if len(centros) > 1:
	# 	sorted_points = sort_points(centros)
	# 	print(sorted_points)

	# 	for i in range(0,len(sorted_points)-1): # Step 2 by 2
	# 		cv2.line(mask_lines, sorted_points[i], sorted_points[i+1], (0,0,255), 3, cv2.LINE_AA)

	# # ### ABORDAGEM 2, USAR A FUNCAO HOUGHLINESP		
	# linesP = cv2.HoughLinesP(edges,rho=2,theta=np.pi/180,threshold=1,minLineLength=10,maxLineGap=1000)
	# if linesP is not None:
	# 		for i in range(0, len(linesP)):
	# 			l = linesP[i][0]
	# 			cv2.line(mask_lines, (l[0], l[1]), (l[2], l[3]), (0,0,255), 3, cv2.LINE_AA)

	# lines = cv2.HoughLines(edges, 1, np.pi/180, threshold=30, min_theta=np.radians(-100), max_theta=np.radians(100))  # Adjust threshold as needed
	# if lines is not None:
	# 	for rho, theta in lines[:, 0]:
					
	# 				if theta > np.pi/2:
	# 					continue
					
	# 				a = np.cos(theta)
	# 				b = np.sin(theta)
	# 				x0 = a * rho
	# 				y0 = b * rho
	# 				x1 = int(x0 + 500 * (-b))
	# 				y1 = int(y0 + 500 * (a))
	# 				x2 = int(x0 - 500 * (-b))
	# 				y2 = int(y0 - 500 * (a))
	# 				cv2.line(mask_lines, (x1, y1), (x2, y2), (255, 0, 0), 2)  # Draw lines in red	


	# ## ABORDAGEM 3, TENTAR OBTER A PARTE CENTRAL DOS CONTORNOS, BASEADO NO STACKOVERFLOW
	# img = imageStreetMark.copy()
	# # print(img.shape)
	# gray = cv2.cvtColor(imageStreetMark.copy(), cv2.COLOR_BGR2GRAY)
	# mask = np.zeros_like(gray)
	# img = gray.copy()
	# size = np.size(img)
	# skel = np.zeros(img.shape,np.uint8)
	# ret,img = cv2.threshold(img,5,255,0)
	# element = cv2.getStructuringElement(cv2.MORPH_CROSS,(3,3))
	# done = False
	# while( not done):
	#     eroded = cv2.erode(img,element)
	#     temp = cv2.dilate(eroded,element)
	#     temp = cv2.subtract(img,temp)
	#     skel = cv2.bitwise_or(skel,temp)
	#     img = eroded.copy() 
	#     zeros = size - cv2.countNonZero(img)
	#     if zeros==size: done = True

	# kernel = np.ones((2,2), np.uint8)
	# skel = cv2.dilate(skel, kernel, iterations=1)
	# skeleton_contours, _ = cv2.findContours(skel, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
	# largest_skeleton_contour = max(skeleton_contours, key=cv2.contourArea)
	# cv2.drawContours(img, largest_skeleton_contour, -1, (255,255,255), 2)

	# # # Extend the skeleton past the edges of the banana
	# points = []
	# for point in largest_skeleton_contour: points.append(tuple(point[0]))
	# x,y = zip(*points)
	# z = np.polyfit(x,y,7)
	# f = np.poly1d(z)

	# print(z,f)
	# x_new = np.linspace(0, img.shape[1],300)
	# y_new = f(x_new)
	# extension = list(zip(x_new, y_new))
	# # img = src.copy()
	# for point in range(len(extension)-1):
	#     a = tuple(np.array(extension[point], int))
	#     b = tuple(np.array(extension[point+1], int))
	#     cv2.line(img, a, b, (0,0,255), 1)
	#     cv2.line(mask, a, b, 255, 1)   
	# mask_px = np.count_nonzero(mask)

	# cv2.imshow("Abordagem3", img)
	# cv2.imshow("Abordagem3Mask", maskss)

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
	cv2.imshow("Skeleton", skeleton)
	skeleton = np.uint8(skeleton)
	edges = cv2.Canny(skeleton, 50, 155)
	linesP = cv2.HoughLinesP(edges,rho=1,theta=0.1*np.pi/180,threshold=20,minLineLength=10,maxLineGap=200)
	lines = []
	if linesP is not None:
		# print(len(linesP))
		for i in range(0, len(linesP)):
			l = linesP[i][0]
			lines.append(l)
			# print(l)
			# cv2.line(mask_lines, (l[0], l[1]), (l[2], l[3]), (255,255,255), 3, cv2.LINE_AA)

		# print(mask_lines.shape)
		sorted_lines = sorted(lines, key=lambda line: min(math.dist((line[0], line[1]), (300, 240)),
                                                  math.dist((line[2], line[3]), (300, 240))))

		# The closest line will be the first element in sorted_lines
		closest_line = sorted_lines[0]

		# If you want to handle cases where x0 and x1 might be inverted, you can rearrange the points
		x0, y0, x1, y1 = closest_line
		if abs(x0 - 300) > abs(x1 - 300):
		    closest_line = (x1, y1, x0, y0)

		# Now closest_line contains the sorted and rearranged line closest to the point (300, 0)
		# print("Closest Line:", closest_line)
		cv2.line(mask_lines, (closest_line[0], closest_line[1]), (closest_line[2], closest_line[3]), (255,255,255), 3, cv2.LINE_AA)



	# cv2.imshow("Output GetRoad", lines_mask)
	# cv2.imshow("Linhas", mask_lines)
	# cv2.imshow("Edges", edges)
	return mask_lines
	# cv2.imshow("Eroded Image", eroded_image)

theta = 0
rho = 300
def getInclination(imageMask):
	global theta,rho
	mask = np.zeros(imageMask.shape, np.uint8)
	# calcula Hough Line Transform
	edges = cv2.Canny(imageMask, 50, 155)
	cv2.imshow("GETINCLINATION", edges)

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
			print(f"DETECTOU: {theta}")

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


	# Desenha a linha central do modelo
	x1_car = int(mask.shape[1]/2)
	y1_car = int(0)
	x2_car = int(mask.shape[1]/2)
	y2_car = int(mask.shape[0])
	# cv2.line(mask, (x1_car, y1_car), (x2_car, y2_car), (255, 255, 255), 2)  # Draw lines in red

	cv2.imshow("Linhas", mask)
	return theta,rho

def getDashedLine(image):
	img = image.copy()
	# img = cv2.cvtColor(image_test, cv2.COLOR_BGR2GRAY)


	kernel1 = np.ones((1,1),np.uint8)
	kernel2 = np.ones((9,9),np.uint8)

	imgGray=cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
	cv2.imshow("imgGray", imgGray) 
	imgBW=cv2.threshold(imgGray, 160, 255, cv2.THRESH_BINARY_INV)[1]
	

	img1=cv2.dilate(imgBW, kernel1, iterations=2)
	img1= cv2.bitwise_not(img1)
	cv2.imshow("img1", img1) 


	contours,_ = cv2.findContours(img1,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)

	# largest_contours = sorted(contours, key=cv2.contourArea)
	selected_contours = []
	for contour in contours:
		area = cv2.contourArea(contour)
		# print(area)
		if area < 600:
			selected_contours.append(contour)
	# print(len(selected_contours))
	mask = np.zeros(img.shape, np.uint8)
	cv2.drawContours(mask, selected_contours, -1, (255,255,255,255), -1)
	


	# Traça a media das linhas
	
	# # calcula Hough Line Transform
	edges = cv2.Canny(mask, 50, 155)
	lines = cv2.HoughLines(edges, 1, np.pi/180, threshold=50, min_theta=np.radians(-60), max_theta=np.radians(60))  # Adjust threshold as needed

	# # desenha linhas

	liness = []
	if lines is not None:
		for rho, theta in lines[:, 0]:
			
			if theta > np.pi/2:
				continue
			


			liness.append( (theta,rho) )
			


		theta = sum([x[0] for x in liness])/len(liness)
		rho = sum([x[1] for x in liness])/len(liness)

		#Theta corresponde ao psi e o rho corresponde ao x

		a = np.cos(theta)
		b = np.sin(theta)
		x0 = a * rho
		y0 = b * rho
		x1 = int(x0 + 1000 * (-b))
		y1 = int(y0 + 1000 * (a))
		x2 = int(x0 - 1000 * (-b))
		y2 = int(y0 - 1000 * (a))


		cv2.line(mask, (x1, y1), (x2, y2), (255, 0, 0), 2)  # Draw lines in red


	# Desenha a linha central do modelo
	x1 = int(mask.shape[1]/2)
	y1 = int(0)
	x2 = int(mask.shape[1]/2)
	y2 = int(mask.shape[0])

	cv2.line(mask, (x1, y1), (x2, y2), (255, 255, 0), 2)  # Draw lines in red
	cv2.imshow("mask", mask) 

	return theta,rho





def detectionApproach1(image):
	# Based on https://medium.com/analytics-vidhya/image-processing-to-detect-curvature-of-road-f68014338778

	#Obtain height and width of image
	h,w,_ = image.shape
	print(h,w)

	# Crop top part to not get too far image
	top = int(h*0.3)
	image_without_top = image[1-top:h, 0:w]

	# Divide image in 3 parts
	delta_lateral = 0.40
	image_esquerda = image[1-top:h,  0:int(w*delta_lateral)]
	image_direita = image[1-top:h,  1-int(w*delta_lateral):w]
	image_central = image[1-top:h,  int(w*delta_lateral):1-int(w*delta_lateral)]

	# print(image_central)
	pixel_cinza = [46,46,46]
	res_image_esquerda =  np.count_nonzero(np.all(image_esquerda==pixel_cinza,axis=2))
	res_image_direita =  np.count_nonzero(np.all(image_direita==pixel_cinza,axis=2))
	# print(f'ESQUERDA: {res_image_esquerda} / DIREITA:{res_image_direita}')


	car.setVel(5.0) # Controle de velocidade basico (cte a 2 m/s)
	if res_image_esquerda > res_image_direita:
		print("VIRA PARA DIREITA")
		car.setSteer(-0.3) # Direcao (Se ver cor azul, joga pro lado direito, se ver verde, joga pro esquerdo)
	elif res_image_direita > res_image_esquerda: 
		car.setSteer(0.3) # Direcao (Se ver cor azul, joga pro lado direito, se ver verde, joga pro esquerdo)
		print("VIRA PARA A ESQUERDA")
	# Conta o numero de pixels cinzas na imagem

	cv2.imshow("CameraWithoutTop",image_without_top)
	cv2.imshow("Left",image_esquerda)
	cv2.imshow("Central",image_central)
	cv2.imshow("Right",image_direita)


def detectionApproach2(image):
	# Detecta as linas e tenta seguir elas de acordo com a angulacao

	#Obtain height and width of image
	h,w,_ = image.shape
	# print(h,w)	

	kernel = np.ones((5,5),np.uint8)


	
	
	
	
	# dilation = cv2.erode(mask_blue,kernel,iterations = 2)



	# blue_dilated = dilateImage(image,mask_blue,kernel_size=60)
	# green_dilated = dilateImage(blue_dilated,mask_green,kernel_size=50,iterations=1)
	#Q07
	# imageStreetMark,selected_contours = getStreetMarks(image)
	# test = getLine(imageStreetMark,selected_contours)
	# image = getDashedLine(image)


	#q10

	# Crop top of image
	top = int(h*0.5)
	image_without_top = image[1-top:h, 0:w]
	imageStreetMark,selected_contours = getStreetMarks(image_without_top)
	mask_lines = getLine(imageStreetMark,selected_contours)
	theta,rho = getInclination(mask_lines)
	# print(theta,rho)
	

	# cv2.imshow("Contorno",dilated)
	# cv2.imshow("Sem Canto", green_dilated)



	# imagewithLines = roadDetection(green_dilated)
	# cv2.imshow("Dilation",imagewithLines)	
	# car.setVel(.5)



	return theta,rho

def detectionApproach3(image):
	image_test = image.copy()
	grey_image = cv2.cvtColor(image_test, cv2.COLOR_BGR2GRAY)
	kernel_size = 5
	blur_gray = cv2.GaussianBlur(grey_image,(kernel_size, kernel_size),0)

	low_threshold = 50
	high_threshold = 200
	edges = cv2.Canny(blur_gray, low_threshold, high_threshold)


	mask = np.zeros_like(edges)
	ignore_mask_color = (255,255,0)
	imshape = image.shape
	vertices = np.array([ [(0,imshape[0]),(450, 200), (500, 200), (imshape[1],imshape[0])] ], dtype=np.int32)
	cv2.fillPoly(mask, vertices, ignore_mask_color)
	masked_edges = cv2.bitwise_and(edges, mask)
	cv2.imshow("BK", mask)

def ControleLateral(car_x,line_x,line_psi,car_psi):
	# k = 1.2 #ganho
	k = 0.8 #ganho
	ke = 0.00392 # Multiplicador do erro
	delta_max = np.deg2rad(20.0) #máximo de guinada

	# cálculo do erro e psi no caso da reta
	erro = ke*(line_x-car_x) # menos a posição x do carro
	psierro = -(line_psi) 
	print(f'erro:{round(erro,5)} / psierro: {round(psierro,2)}')

	#Controle Stanley
	acao_guinada = psierro + np.arctan(( k * erro) / car.v)
	if np.abs(acao_guinada) < delta_max: 
	    delta = acao_guinada 		
	else:
		if acao_guinada >= delta_max:#saturou positivamente
			delta = delta_max
		else: # saturou negativamente acao_guinada <= -delta_max:
			delta = -delta_max

	return delta

fig = plt.figure()
plt.imshow(mapa, extent=[-7.5, 7.5, -7.5, 7.5], alpha=0.99)
grafico, = plt.plot(x,y)

# image = cv2.imread('../Imagens Teste/Road4_Miss_Detection.png') # IMAGEM FIXA
# image = cv2.imread('../Imagens Teste/Road5_Distance.png') # IMAGEM FIXA
while car.t < 200:
	
	# lê sesnores
	car.step()

	# Pega a camera
	image = car.getImage()

	image = cv2.flip(cv2.cvtColor(image, cv2.COLOR_BGR2RGB), 0) # Inverte a imagem

	# #blobs
	# image,green_area,_,_,_ = blob(image,GREEN_HSV_MIN,GREEN_HSV_MAX, color=(0,255,0))
	# image,blue_area,_,_,_ = blob(image,BLUE_HSV_MIN,BLUE_HSV_MAX, color=(0,255,255))
	# image,white_area,cx_white,cy_white,_ = blob(image,WHITE_HSV_MIN,WHITE_HSV_MAX, color=(255,255,255))

	# if green_area > blue_area: # Se tiver mais verde que azul
	# 	delta = -5.0
	# else:
	# 	delta = +5

	# atua
	# car.setSteer(delta) # Direcao (Se ver cor azul, joga pro lado direito, se ver verde, joga pro esquerdo)

	# car.setU(np.abs(np.sin(0.01*car.getTime())))
	# car.setVel(2.0) # Controle de velocidade basico (cte a 2 m/s)
	

	# Armazena valores de posicao e tempo do carrinho
	x.append(car.p[0])
	y.append(car.p[1])
	time.append(car.t)


	# lê e exibe camera
	
	cv2.imshow("Camera",image)


	# detectionApproach1(image)
	theta,line_x = detectionApproach2(image)

	delta = ControleLateral(300,line_x=line_x,line_psi=theta,car_psi=0)


	print('DELTA:',delta)
	car.setSteer(delta) 
	# car.setVel(0.1)
	car.setVel(0.3)
	# Live graph -> FUNCAO LENTA
	img_grafico = updateGraph(fig,grafico,x,y)
	cv2.imshow("Grafico",img_grafico)

	# Exibe carrinho e posicao no mapa


	if cv2.waitKey(1) & 0xFF == ord('q'):
		break
	
print('Terminou...')
