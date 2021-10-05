#Team 4 Math

import numpy as np


#Finds an intersection of f1 and f2 in the interval [x_i, x_f], assumes there is exactly one intersection
def FindIntersection(f1, f2, x_i, x_f, prec = 0.0001):
	x_a = x_i
	x_b = x_f
	x_m = (x_a + x_b)/2

	if (np.absolute(f1(x_a) - f2(x_a)) <= prec):
		return x_a, ((f1(x_a)+f2(x_a))/2)
	if (np.absolute(f1(x_b) - f2(x_b)) <= prec):
		return x_b, ((f1(x_b)+f2(x_b))/2)
	if (np.absolute(f1(x_m) - f2(x_m)) <= prec):
		return x_m, ((f1(x_m)+f2(x_m))/2)

	if (f1(x_a) > f2(x_a)):
		if(f1(x_m) < f2(x_m)): x_b = x_m
		else: x_a = x_m
	else:
		if(f1(x_m) > f2(x_m)): x_b = x_m
		else: x_a = x_m

	return FindIntersection(f1, f2, x_a, x_b)
#!FindIntersection


#Return the largest of the two functions f1 and f2 at a point x
def FindMax(f1, f2, x):
	if (f1(x) >= f2(x)): 
		return f1
	else: 
		return f2
#!FindMax