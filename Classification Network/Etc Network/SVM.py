import pandas as pd
import numpy as np
import pickle
import matplotlib.pyplot as plt
from sklearn.model_selection import train_test_split
from sklearn.svm import SVC
from sklearn.metrics import classification_report, confusion_matrix

#Loading the dataset
data = pd.read_csv("Data_10.csv")

FFT_Hz = 10+1
train_min = 6500
train_max = 9000
test_min = 9500
test_max = 11500
epoch = 10

# Loading data
data = np.loadtxt("Data_10.csv", delimiter=",")

train_sample_num = train_max - train_min

#data_FFTCov = data[:, 0:FFT_Hz]
#data_max_FFT = data_FFTCov[:, 0:FFT_Hz-1].max()  
#data_max_Cov = data_FFTCov[:, FFT_Hz-1].max()
#data[:, 0:FFT_Hz-1] = data[:, 0:FFT_Hz-1]/data_max_FFT
#data[:, FFT_Hz-1] = data[:, FFT_Hz-1]/data_max_Cov

# Filtering data
x_train_data = data[train_min:train_max, 0:FFT_Hz]
#np.random.shuffle(x_train_data)
y_train_data = data[train_min:train_max, FFT_Hz]
y_train_data = y_train_data.reshape(train_sample_num, 1)
#np.random.shuffle(y_train_data)

# Preprocessing Y data
for i in range(train_max-train_min):
	if y_train_data[i] != 0.0:
		y_train_data[i] = 1
	else:
		y_train_data[i] = 0

# Organizing test data
x_test_data = data[test_min:test_max, 0:FFT_Hz]
y_test_data = data[test_min:test_max, FFT_Hz]

for i in range(test_max-test_min):
	if y_test_data[i] != 0.0:
		y_test_data[i] = 1
	else:
		y_test_data[i] = 0


#Linear Kernel (Best fit for single channels)
#Conducting the fitting for linear kernel
svclassifier_linear = SVC(kernel = 'linear', verbose=True)
svclassifier_linear.fit(x_train_data, y_train_data)
y_pred_linear = svclassifier_linear.predict(x_test_data)

print('Linear Kernel:')
print(confusion_matrix(y_test_data, y_pred_linear))
print(classification_report(y_test_data, y_pred_linear))

#Polynomial Kernel
svclassifier_poly = SVC(kernel = 'poly', degree = 8)
svclassifier_poly.fit(x_train_data,y_train_data)
y_pred_poly = svclassifier_poly.predict(x_test_data)

print('Polynomial Kernel:')
print(confusion_matrix(y_test_data, y_pred_poly))
print(classification_report(y_test_data, y_pred_poly))

#Gaussian Kernel
svclassifier_G = SVC(kernel = 'rbf')
svclassifier_G.fit(x_train_data, y_train_data)
y_pred_G = svclassifier_G.predict(x_test_data)

print('Gaussian Kernel:')
print(confusion_matrix(y_test_data, y_pred_G))
print(classification_report(y_test_data, y_pred_G))

#Sigmoid
svclassifier_sigmoid = SVC(kernel = 'sigmoid')
svclassifier_sigmoid.fit(x_train_data, y_train_data)
y_pred_sigmoid = svclassifier_sigmoid.predict(x_test_data)

print('Sigmoid:')
print(confusion_matrix(y_test_data, y_pred_sigmoid))
print(classification_report(y_test_data, y_pred_sigmoid))

#Saving the model
filename = 'SVM_linear_kernel.sav'
pickle.dump(svclassifier_linear,open(filename,'wb'))
