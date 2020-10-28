import numpy as np

w1 = .01
w2 = -5
b = -1
t = 1
x = 3

def sigmoid (x):
	return 1 / (1 + np.exp(-x))

def deriv_sigmoid (x):
	return sigmoid(x) * (1 - sigmoid(x))

def z1():
	return x * w1

def a1():
	print(type(z1()))
	print(type(0.0))
	return np.max([0.0, z1()])

def z2():
	return w2 * a1() + b

def y():
	return sigmoid(z2())

def C():
	return .5 * (y() - t) ** 2


print((y()-t) * deriv_sigmoid(z2()) * w2 * z1() * x)