import numpy as np

def sigmoid (input):
	return 1 / (1 + np.exp(-input))

fh = 0
ih = 0
oh = 0
fx = 0
ix = 100
ox = 100
bf = -100
bi = 100
bo = 0
ch = -100
cx = 50
bc = 0
hidden_state = 0
visible_state = 0

def f(x):
	return sigmoid(fh * hidden_state + fx * x + bf)

def i(x):
	return sigmoid(ih * hidden_state + ix * x + bi)

def o(x):
	return sigmoid(oh * hidden_state + ox * x + bo)

def cur(x):
	global visible_state
	visible_state = f(x) * visible_state + i(x) * np.tanh(ch * hidden_state + cx * x + bc)
	return visible_state

def hid(x):
	global hidden_state
	hidden_state = o(x) * np.tanh(cur(x))
	return hidden_state

hidden = []
for x in [1, 1, 0, 1, 1]:
	hidden.append(hid(x))

#print(hidden)
print(sigmoid(-5.97)*(1-sigmoid(-5.97)))