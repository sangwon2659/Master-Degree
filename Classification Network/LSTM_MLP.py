import numpy as np
import matplotlib.pyplot as plt
from keras import Sequential
from keras.layers import LSTM
from keras.layers import RepeatVector
from keras.layers import TimeDistributed
from keras.layers import Dense
from keras.layers import Input
from keras.layers import Dropout
from tensorflow import keras
from pandas import read_csv

###################### Parameter Variables ######################

FFT_Hz = 40+1
train_min = 0
train_max = 13200
test_min = 8000
test_max = 13200
epoch = 100

#################################################################

################ Loading and Normalizing Raw Data ################

#Loading Data
data = read_csv("Training_Data_Plastic.csv", header=None)
data = np.array(data)

#Stating necessary variables
total_samples = data.shape[0]
train_samples = train_max - train_min
test_samples = test_max - test_min

#Preparation for FFT data normalization
data_FFT = data[:, 0:FFT_Hz]
#data_FFT_max, data_FFT_min = data_FFT.max(), data_FFT.min()
#data_FFT_max_min_diff = data_FFT_max - data_FFT_min

#Slip data into binary classes
for i in range(total_samples):
	if data[i,FFT_Hz] != 0.0:
		data[i,FFT_Hz] = 1.0

##################################################################

###################### Preprocessing Data ########################

#Declaring training data
training_data = data[train_min:train_max, :]
#np.random.shuffle(training_data)

#Declaring x_train and y_train data
x_train_data, y_train_data = training_data[:, 0:FFT_Hz], training_data[:, FFT_Hz]

#Data normalization for x_train data
#x_train_data = (x_train_data/data_FFT_max_min_diff)*100

#Reshaping x_train and y_train data
x_train_data = x_train_data.reshape(train_samples, FFT_Hz, 1)
y_train_data = y_train_data.reshape(train_samples, 1)

#Declaring test data
test_data = data[test_min:test_max, :]
#np.random.shuffle(test_data)

#Declaring x_test and y_test data
x_test_data, y_test_data = test_data[:, 0:FFT_Hz], test_data[:, FFT_Hz]

#Data normalization for x_test data
#x_test_data = (x_test_data/data_FFT_max_min_diff)*100

#Reshaping x_test and x_test data
x_test_data = x_test_data.reshape(test_samples, FFT_Hz, 1)
y_test_data = y_test_data.reshape(test_samples, 1)
print(x_test_data)
print(y_test_data)

##################################################################

######################## Defining Model ##########################

#Model is a LSTM combined with MLP for binary slip classification purposes

model = Sequential()
model.add(LSTM(500, activation='relu', input_shape=(FFT_Hz, 1)))
model.add(Dense(1024, activation = 'relu'))
model.add(Dropout(0.3))
model.add(Dense(512, activation = 'relu'))
model.add(Dropout(0.3))
model.add(Dense(256, activation = 'relu'))
model.add(Dropout(0.3))
model.add(Dense(64, activation = 'relu'))
model.add(Dropout(0.3))
model.add(Dense(32, activation = 'relu'))
model.add(Dense(1, activation = 'sigmoid'))

model.summary()

model.compile(optimizer='adam', loss='binary_crossentropy', metrics=['accuracy'])

'''
#Defining input shape
visible = Input(shape=(FFT_Hz,1))
#LSTM model of shapes 256, 128 ,64, 1
LSTM_model = LSTM(256, activation='relu', input_shape=(FFT_Hz,1), return_sequences=True)(visible)
LSTM_model = LSTM(128, activation='relu', return_sequences=True)(LSTM_model)
LSTM_model = LSTM(64, activation='relu', return_sequences=True)(LSTM_model)
#LSTM_model = TimeDistributed(Dense(1))(LSTM_model)
#MLP model of shapes 1024, 512, 256 and 1
MLP_model = Dense(1024, activation='relu')(LSTM_model)
MLP_model = Dropout(0.3)(MLP_model)
MLP_model = Dense(256, activation='relu')(MLP_model)
MLP_model = Dropout(0.3)(MLP_model)
MLP_model = Dense(64, activation='relu')(MLP_model)
MLP_model = Dropout(0.3)(MLP_model)
MLP_model = Dense(1, activation='softmax')(MLP_model)
#Compiling model
model = keras.Model(inputs=visible, outputs=MLP_model)
model.compile(optimizer='adam', loss='mse', metrics=['accuracy'])
#Printing a summary of the model
model.summary()
'''

#################################################################

################# Fitting and Evaluating Model ##################

#Fitting model and saving into history
history = model.fit(x_train_data, y_train_data, epochs = epoch, verbose=1)

#Plotting loss
plt.plot(history.history['loss'])
plt.title('Model Loss')
plt.ylabel('Loss')
plt.xlabel('Epoch')
plt.show()

plt.plot(history.history['accuracy'])
plt.title('Model Accuracy')
plt.ylabel('Accuracy')
plt.xlabel('Epoch')
plt.show()

#Evaluating model with test data
evaluation = model.evaluate(x_test_data, y_test_data)

##################################################################
