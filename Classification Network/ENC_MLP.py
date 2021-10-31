import numpy as np
import matplotlib.pyplot as plt
import keras
from keras.layers import Dense
from keras.layers import Dropout
from keras.layers import TimeDistributed
from keras.layers import Permute
from keras.layers import Flatten
from sklearn.metrics import accuracy_score
from tensorflow.keras.models import load_model
from pandas import read_csv

##################### Parameter Variables ######################

n_features = 40+1
train_min = 0
train_max = 13200
test_min = 10000
test_max = 13200
timesteps = 20
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
data_ = data[:, 0:n_features]
#data_FFT_max, data_FFT_min = data_FFT.max(), data_FFT.min()
#data_FFT_max_min_diff = data_FFT_max - data_FFT_min

#Slip data into binary classes
for i in range(total_samples):
        if data[i,n_features] != 0.0:
                data[i,n_features] = 1.0

##################################################################

###################### Preprocessing Data ########################

'''
def temporalize(X, y, lookback):
    output_X = []
    output_y = []
    for i in range(len(X)-lookback-1):
        t = []
        for j in range(1,lookback+1):
            # Gather past records upto the lookback period
            t.append(X[[(i+j+1)], :])
        output_X.append(t)
        output_y.append(y[i+lookback+1])
    return output_X, output_y
'''

#Declaring training data
training_data = data[train_min:train_max, :]
#np.random.shuffle(training_data)

#Declaring x_train and y_train data
x_train_data, y_train_data = training_data[:, 0:n_features], training_data[:, n_features]
#x_train_data, y_train_data_ = temporalize(x_train_data, y_train_data, timesteps)
#x_train_data = np.array(x_train_data)
x_train_data = x_train_data.reshape(train_samples, n_features, 1)

#Reshaping x_train and y_train data
#x_train_data = x_train_data.reshape(train_samples, n_features, 1)
y_train_data = y_train_data.reshape(train_samples, 1)
#y_train_data = y_train_data[21:,:]

#Declaring test data
test_data = data[test_min:test_max, :]
#np.random.shuffle(test_data)

#Declaring x_test and y_test data
x_test_data, y_test_data = test_data[:, 0:n_features], test_data[:, n_features]
#x_test_data, y_test_data_ = temporalize(x_test_data, y_test_data, timesteps)
#x_test_data = np.array(x_test_data)
x_test_data = x_test_data.reshape(test_samples, n_features, 1)

#Data normalization for x_test data
#x_test_data = (x_test_data/data_FFT_max_min_diff)*100

#Reshaping x_test and x_test data
#x_test_data = x_test_data.reshape(test_samples, n_features, 1)
y_test_data = y_test_data.reshape(test_samples, 1)

### Loading encoder ###
encoder = load_model('encoder.h5')
x_train_enc = encoder.predict(x_train_data)
print(np.shape(x_train_enc))
print(x_train_enc)

### Creating the MLP model ###
# MLP with 1024 -> 512 -> 256 -> 2 layers
model = keras.Sequential()
model.add(Dense(1024, activation='relu', input_shape=(n_features, 64)))
model.add(Dense(512, activation='relu'))
model.add(Dense(256, activation='relu'))
model.add(Dense(64, activation='relu'))
model.add(Dense(32, activation='relu'))
model.add(Flatten())
model.add(Dense(1, activation='sigmoid'))
model.compile(loss='binary_crossentropy', optimizer='adam', metrics=['binary_accuracy']) 
# Extracting a summary of the model
model.summary()

# Defining history
history = {"loss": [], "accuracy": []};
history = model.fit(x_train_enc, y_train_data, epochs=epoch, verbose=1)

'''
### Fitting data into the model ###
Training_data_row_num = 50
epochs_ = 30
for i in range(Training_data_row_num):
	# Reshaping data to be put into encoder
	x_train_data_temp = x_train_data[0,i,0:500].reshape(1,1,500)
	x_train_enc = encoder.predict(x_train_data_temp)
	if y_train_data[0,0,i] == 0.0:
		# [[[1,0]]] == slip
		y_train_data_temp = np.array([[[0, 1]]])
	else:
		y_train_data_temp = np.array([[[1, 0]]])	
	history_temp = model.fit(x_train_enc, y_train_data_temp, epochs = epochs_, verbose = 1)
	print(model.predict(x_train_enc))
	# Adding history_temp data to history
        # Extend function adds the history_temp list as single elements
	history["loss"].extend(history_temp.history['loss'])
	#history["accuracy"].append()
'''

### Plotting history ###
# Plotting loss
plt.plot(history["loss"])
plt.title('Model Loss')
plt.ylabel('Loss')
plt.xlabel('Epoch')
plt.show()

plt.plot(history["accuracy"])
plt.title('Model Accuracy')
plt.ylabel('Accuracy')
plt.xlabel('Epoch')
plt.show()

'''
# Prediction from encoded test data through the MLP model
for j in range(500):
	x_test_data_temp = x_test_data[0,j,0:500].reshape(1,1,500)
	x_test_enc = encoder.predict(x_test_data_temp)
	yhat = model.predict(x_test_enc)
	print("groundtruth: {}".format(y_test_data[0,0,j]))
	print("yhat: {}".format(yhat))
'''

# Printing accuracy using the y test data
#acc = accuracy_score(y_test_data, yhat)
#print(acc)

x_test_enc = encoder.predict(x_test_data)

# Evaluating results
evaluation = model.evaluate(x_test_enc, y_test_data)
