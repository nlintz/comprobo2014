from pickle import load
import sklearn
import numpy as np
from random import sample
import pdb
from sklearn.ensemble import RandomForestClassifier
from sklearn.linear_model import LogisticRegression
from sklearn.cross_validation import StratifiedKFold
from sklearn.cluster import KMeans
from sklearn import svm, grid_search

k = 5

f = open('SIFT_features.pickle','r')
descriptors = load(f)
f.close()

X = np.zeros((0,128))
y = np.zeros((0,))

categories = descriptors.keys()
total_descriptors = 0
total_samples = 0
for i in range(len(categories)):
	for data in descriptors[categories[i]]:
		total_descriptors += data.shape[0]
		total_samples += 1

X = np.zeros((total_descriptors,128))
curr = 0
for i in range(len(categories)):
	print i
	for data in descriptors[categories[i]]:
		X[curr:curr+data.shape[0]] = data
		curr += data.shape[0]

fit_X = X[sample(range(total_descriptors),10000),:]
clusters = KMeans(n_clusters=k)
clusters.fit(fit_X)

curr = 0
X_transformed = np.zeros((total_samples,k))
y = np.zeros((total_samples,1))
for i in range(len(categories)):
	for data in descriptors[categories[i]]:
		assignments = clusters.predict(data)
		counts,dc = np.histogram(assignments,bins=np.arange(0,k+1)-0.5)
		X_transformed[curr,:] = counts/float(sum(counts))
		y[curr] = i
		curr += 1

param_grid = [
		{'C': [1, 10, 100, 1000, 10000], 'kernel': ['linear']},
		{'C': [1, 10, 100, 1000], 'gamma': [0.001, 0.0001], 'kernel': ['rbf']},
		]
y = np.ravel(y)
skf = StratifiedKFold(y,5)
for train, test in skf:
	model = grid_search.GridSearchCV(svm.SVC(), param_grid)
	model.fit(X_transformed[train,:],y[train])
	print model.score(X_transformed[test,:],y[test])
