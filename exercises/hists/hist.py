import cv2
import numpy as np
from matplotlib import pyplot as plt


""" TWO WAYS TO CALCULATE HISTOGRAMS:
	cv2.calcHist(images, channels, mask, histSize, ranges[, hist[, accumulate]])
"""

cliffsColor = cv2.imread('cliffs.jpg')
cliffsGray = cv2.imread('cliffs.jpg', 0)
snowyGray = cv2.imread('snowy.jpg', 0)

def colorHistograms(img):
	color = ('b','g','r')
	for i,col in enumerate(color):
	    histr = cv2.calcHist([img],[i],None,[256],[0,256])
	    plt.plot(histr,color = col)
	    plt.xlim([0,256])
	plt.show()

def histEqualization(img):
	equ = cv2.equalizeHist(img)
	res = np.hstack((img,equ)) #stacking images side-by-side
	cv2.imwrite('res.png',res)

def CLAHE(img):
	clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
	cl1 = clahe.apply(img)

	cv2.imwrite('clahe_2.jpg',cl1)

colorHistograms(cliffsColor)
histEqualization(snowyGray)
CLAHE(snowyGray)

