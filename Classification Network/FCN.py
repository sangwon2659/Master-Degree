input_layer = keras.layers.Input(input_shape)
conv1 = keras.layers.Conv1d(filters=128, kernel_size=8, padding='same')(input_layer)
conv1 = keras.layers.BatchNormalization()(conv1)
conv1 = keras.layers.Activation(activation='relu')(conv1)


conv2 = keras.layers.Conv1d(filters=256, kernel_size=5, padding='same')(conv1)
conv2 = keras.layers.BatchNormalization()(conv2)
conv2 = keras.layers.Activation(activation='relu')(conv2)

conv3 = keras.layers.Conv1d(filters=128, kernel_size=3, padding='same')(conv2)
conv3 = keras.layers.BatchNormalization()(conv3)
conv3 = keras.layers.Activation(activation='relu')(conv3)

gap_layer = keras.layers.Global.AveragePooling1D()(conv3)

output_layer = keras.layers.Dense(nb_classses, activation='softmax')(gap_layer)

model = keras.models.Model(inputs=input_layer, outputs=output_layer)

model.compile(loss'categorical_crossentropy', optimizer = keras.optimizers.Adam(), metrics=['accuracy'])
