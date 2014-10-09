from scipy.stats import norm
from matplotlib import pyplot
import numpy as np
import sys

def plotGaussian(mean, sigma, x):
	y = map(lambda x:norm(mean, sigma).pdf(x), x)
	pyplot.plot(x, y)
	pyplot.show()

def main():
	if len(sys.argv) == 5:
		mean = float(sys.argv[1])
		sigma = float(sys.argv[2])
		xmin = float(sys.argv[3])
		xmax = float(sys.argv[4])
	else:
		mean = 0
		sigma = 1
		xmin = -1
		xmax = 1

	plotGaussian(mean, sigma, np.linspace(xmin, xmax, 1000))

if __name__ == "__main__":
	main()