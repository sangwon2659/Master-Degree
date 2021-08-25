import numpy as np
import tensorflow as tf
from keras import optimizers
from numpy import array
from keras.models import Sequential
from keras.layers import Dense, LSTM

FFT_Hz = 50
train_min = 0
train_max = 7862
test_min = 6500
test_max = 7500
epoch = 100

# Loading data
data = np.loadtxt("Data_Realtime.csv", delimiter=",")

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

print(x_train_data)
print(y_train_data)

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

print(np.shape(x_test_data[0]))

# Defining model
model = Sequential()
model.add(Dense(1024, input_shape=(FFT_Hz,), activation='relu'))
model.add(Dense(512, activation='relu'))
model.add(Dense(256, activation='relu'))
model.add(Dense(64, activation='relu'))
model.add(Dense(32, activation='relu'))
model.add(Dense(16, activation='relu'))
model.add(Dense(1, activation='sigmoid'))

model.summary()

model.compile(loss='binary_crossentropy', optimizer='adam', metrics=['accuracy'])
history = model.fit(x_train_data, y_train_data, epochs=epoch, batch_size=10, verbose=1)

# Evaluating results
evaluation = model.evaluate(x_test_data, y_test_data)

x_test_data_ = x_test_data[0]
x_test_data_ = x_test_data_.reshape(1,FFT_Hz)
#print(np.shape(x_test_data_))
#print(x_test_data_)
#print(np.shape(x_test_data))
#print(x_test_data)
prediction = model.predict(x_test_data_)
print(prediction)

model.save("MLP_Model.h5")
