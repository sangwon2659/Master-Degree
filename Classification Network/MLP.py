import numpy as np
import tensorflow as tf
import matplotlib.pyplot as plt
from keras import optimizers
from numpy import array
from keras.models import Sequential
from keras.layers import Dense, LSTM
import pandas as pd

FFT_Hz = 41
train_min = 0
train_max = 5961
test_min = 5000
test_max = 5961
epoch = 500

# Loading data
data = np.loadtxt("Training_Data_Aluminum.csv", delimiter=",")

train_sample_num = train_max - train_min
test_sample_num = test_max - test_min

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

# Displaying loss history
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

# Evaluating results
evaluation = model.evaluate(x_test_data, y_test_data)
print("Test Loss: // Test Accuracy:")
print(evaluation)

df = pd.DataFrame(history.history['accuracy'])
df.to_csv('Accuracy_Paper.csv')

model.save("MLP_Model_Paper.h5")
