import numpy as np
import matplotlib.pyplot as plt
import time
import random

class filter():
	def __init__(self, mean0, var0, meanSensor, varSensor, meanMove, varMove, positions, distances):
		self.mean0 = mean0
		self.var0 = var0
		self.meanSensor = meanSensor
		self.varSensor = varSensor
		self.positions = positions
		self.distances = distances
		self.meanMove = meanMove
		self.varMove  = varMove

	def get_prediction(self):
		new_var, new_mean = self.predict(self.var0, self.mean0, self.varMove, self.meanMove)
		var, mean = self.correct(new_var, new_mean, self.varSensor, self.meanSensor)
		vals=[]
		for m in range(len(positions)):
			var, mean = self.predict(var, mean, self.varMove, self.distances[m])
			var, mean = self.correct(var, mean, self.varSensor, self.positions[m])
			vals.append([var, mean])
	

		return vals[-1]

	def predict(self, var, mean, varMove, meanMove):
		new_var = var + varMove
		new_mean= mean+ meanMove
		return new_var, new_mean

	def correct(self, var, mean, varSensor, meanSensor):
		new_mean=(varSensor*mean + var*meanSensor) / (var+varSensor)
		new_var = 1/(1/var +1/varSensor)
		return new_var, new_mean

positions = [0] 
distances = [0]

predictions = []
variances = []

fixes_x = []
fixes_y = []

for numb in range(0, 130):
	position = random.random()*10
	positions.append(position)

	distance = position-distances[-1]
	distances.append(distance)

	meanSensor = sum(positions)/len(positions)	
	varSensor  = sum((i - meanSensor) ** 2 for i in positions) / len(positions) 

	meanMove = sum(distances)/len(distances)	
	varMove  = sum((i - meanMove) ** 2 for i in distances) / len(distances) 

	ft = filter(0, 0, meanSensor, varSensor, meanMove, varMove, positions, distances)
	pred=ft.get_prediction()

	predictions.append(pred[1])
	variances.append(pred[0])

	if pred[0] > 8.1:
		fixes_x.append(pred[0])
		fixes_y.append(pred[1])
