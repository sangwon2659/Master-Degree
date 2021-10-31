import pandas as pd
import numpy as np
import pickle
import matplotlib.pyplot as plt
from sklearn.model_selection import train_test_split
from sklearn.svm import SVC
from sklearn.metrics import classification_report, confusion_matrix
from sklearn.linear_model import LogisticRegression

#Loading the dataset
data = pd.read_csv("Training_Data_Paper.csv")

FFT_Hz = 40+1
train_min = 0
train_max = 5961
test_min = 4500
test_max = 5961
epoch = 100

# Loading data
data = np.loadtxt("Training_Data_Aluminum.csv", delimiter=",")

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
y_train_data = y_train_data.reshape(train_sample_num, )
print(y_train_data)
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

model = LogisticRegression()
model.fit(x_train_data, y_train_data)
y_pred_Logistic = model.predict(x_test_data)

print('Logistic Regression:')
print(confusion_matrix(y_test_data, y_pred_Logistic))
print(classification_report(y_test_data, y_pred_Logistic))
