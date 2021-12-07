import sys
import numpy as np
import tensorflow as tf
import tensorflow.keras as keras
import matplotlib.pyplot as pyplot
from keras import optimizers
from numpy import array
from keras.models import Sequential
from tensorflow.keras.layers import Conv2D, BatchNormalization, Activation,ZeroPadding2D, MaxPool2D, Add, GlobalAveragePooling2D,Dense
import pandas as pd
from tensorflow.keras.models import Model
from tensorflow.keras import Input

# set parameters
input_data_num = 10
train_min = 0
train_max = 5961
test_min = 5000
test_max = 5961
train_sample_num = train_max - train_min
test_sample_num = test_max - test_min
epoch = 5

# load train and test dataset
def load_dataset():
    # load from csv file
    data = np.loadtxt("Test.csv", delimiter=",")
    # reshape train data
    x_train_data = data[train_min:train_max, 0:input_data_num]
    x_train_data = x_train_data.reshape(train_sample_num, input_data_num, 1)
    y_train_data = data[train_min:train_max, input_data_num]
    y_train_data = y_train_data.reshape(train_sample_num, )
    for i in range(train_max-train_min):
            if y_train_data[i] != 0.0:
                    y_train_data[i] = 1
            else:
                    y_train_data[i] = 0
    # reshape test data
    x_test_data = data[test_min:test_max, 0:input_data_num]
    x_test_data = x_test_data.reshape(test_sample_num, input_data_num, 1)
    y_test_data = data[test_min:test_max, input_data_num]
    y_test_data = y_test_data.reshape(test_sample_num, )
    for i in range(test_max-test_min):
            if y_test_data[i] != 0.0:
                    y_test_data[i] = 1
            else:
                    y_test_data[i] = 0
    return x_train_data, y_train_data, x_test_data, y_test_data

def build_model():
    n_feature_maps = 64
    input_shape=(input_data_num, 1)
    nb_classes = 1

    input_layer = keras.layers.Input(input_shape)

    # BLOCK 1

    conv_x = keras.layers.Conv1D(filters=n_feature_maps, kernel_size=8, padding='same')(input_layer)
    conv_x = keras.layers.BatchNormalization()(conv_x)
    conv_x = keras.layers.Activation('relu')(conv_x)

    conv_y = keras.layers.Conv1D(filters=n_feature_maps, kernel_size=5, padding='same')(conv_x)
    conv_y = keras.layers.BatchNormalization()(conv_y)
    conv_y = keras.layers.Activation('relu')(conv_y)

    conv_z = keras.layers.Conv1D(filters=n_feature_maps, kernel_size=3, padding='same')(conv_y)
    conv_z = keras.layers.BatchNormalization()(conv_z)

    # expand channels for the sum
    shortcut_y = keras.layers.Conv1D(filters=n_feature_maps, kernel_size=1, padding='same')(input_layer)
    shortcut_y = keras.layers.BatchNormalization()(shortcut_y)

    output_block_1 = keras.layers.add([shortcut_y, conv_z])
    output_block_1 = keras.layers.Activation('relu')(output_block_1)

    # BLOCK 2

    conv_x = keras.layers.Conv1D(filters=n_feature_maps * 2, kernel_size=8, padding='same')(output_block_1)
    conv_x = keras.layers.BatchNormalization()(conv_x)
    conv_x = keras.layers.Activation('relu')(conv_x)

    conv_y = keras.layers.Conv1D(filters=n_feature_maps * 2, kernel_size=5, padding='same')(conv_x)
    conv_y = keras.layers.BatchNormalization()(conv_y)
    conv_y = keras.layers.Activation('relu')(conv_y)

    conv_z = keras.layers.Conv1D(filters=n_feature_maps * 2, kernel_size=3, padding='same')(conv_y)
    conv_z = keras.layers.BatchNormalization()(conv_z)

    # expand channels for the sum
    shortcut_y = keras.layers.Conv1D(filters=n_feature_maps * 2, kernel_size=1, padding='same')(output_block_1)
    shortcut_y = keras.layers.BatchNormalization()(shortcut_y)

    output_block_2 = keras.layers.add([shortcut_y, conv_z])
    output_block_2 = keras.layers.Activation('relu')(output_block_2)

    # BLOCK 3

    conv_x = keras.layers.Conv1D(filters=n_feature_maps * 2, kernel_size=8, padding='same')(output_block_2)
    conv_x = keras.layers.BatchNormalization()(conv_x)
    conv_x = keras.layers.Activation('relu')(conv_x)

    conv_y = keras.layers.Conv1D(filters=n_feature_maps * 2, kernel_size=5, padding='same')(conv_x)
    conv_y = keras.layers.BatchNormalization()(conv_y)
    conv_y = keras.layers.Activation('relu')(conv_y)

    conv_z = keras.layers.Conv1D(filters=n_feature_maps * 2, kernel_size=3, padding='same')(conv_y)
    conv_z = keras.layers.BatchNormalization()(conv_z)

    # no need to expand channels because they are equal
    shortcut_y = keras.layers.BatchNormalization()(output_block_2)

    output_block_3 = keras.layers.add([shortcut_y, conv_z])
    output_block_3 = keras.layers.Activation('relu')(output_block_3)

    # FINAL

    gap_layer = keras.layers.GlobalAveragePooling1D()(output_block_3)

    output_layer = keras.layers.Dense(nb_classes, activation='softmax')(gap_layer)

    model = keras.models.Model(inputs=input_layer, outputs=output_layer)

    model.compile(loss='binary_crossentropy', optimizer=keras.optimizers.Adam(),
                  metrics=['accuracy'])

    reduce_lr = keras.callbacks.ReduceLROnPlateau(monitor='loss', factor=0.5, patience=50, min_lr=0.0001)

    return model

# run the test harness for evaluating a model
def run_test_harness():
    # load dataset
    trainX, trainY, testX, testY = load_dataset()
    # define model
    model = build_model()
    # fit model
    history = model.fit(trainX, trainY, epochs=epoch, batch_size=64, validation_data=(testX, testY), verbose=2)
    # evaluate model
    _, acc = model.evaluate(testX, testY, verbose=0)
    print('> %.3f' % (acc * 100.0))
    # learning curves
    summarize_diagnostics(history)
    # save display results
    df = pd.DataFrame(history.history['accuracy'])
    filename = sys.argv[0].split('/')[-1]
    df.to_csv(filename + '_accuracy.csv')
    # save learned model
    model.save('Raw_ResNet.h5')

# entry point, run the test harness
run_test_harness()

