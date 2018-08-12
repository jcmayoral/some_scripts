import numpy as np
from sklearn.svm import SVC
X = np.array([[-1, -1], [-2, -1], [1, 1], [2, 1],[0,0]])
y = np.array(['a', 'a', 'b', 'b','c'])
clf = SVC()
clf.fit(X, y) 

X = np.array([[0, 1]])
y = np.array('d')

A = np.array([[0,0],[0,1]])
print clf.decision_function(X)
print clf.predict(A)
