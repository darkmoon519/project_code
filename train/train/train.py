import matplotlib.pyplot as plt
import numpy as np
import PIL
import tensorflow as tf
import os
import pathlib
import random

from tensorflow import keras
from tensorflow.keras import layers
from tensorflow.keras.models import Sequential
from os.path import exists
from google.colab import files

BATCH_SIZE = 8
INPUT_HEIGHT = 32
INPUT_WIDTH = 32
INPUT_CHANNEL = 3
NUM_CLASSES = 36

def resize_file_data(data):
	data_flattened = data.split(', ')
	for i in range(0, len(data_flattened)):
	data_flattened[i] = int(data_flattened[i])
	# countOfData = len(data_flattened)
	# print(countOfData)
	data = np.resize(data_flattened, (INPUT_HEIGHT, INPUT_WIDTH, INPUT_CHANNEL))
	# print(data)
	return data

def file_dataset_from_directory(data_path, data_type):
	# Get train, valid and test data from files
	data = []
	label = []
	my_dataset_data = []
	my_dataset_label = []

	for name in class_names:
		# print(name)
		data_dir = data_path / pathlib.Path(name)
		for filename in os.listdir(data_dir):
			file_dir = os.path.join(data_dir, filename)
			# Checking if it is a file
			if os.path.isfile(file_dir):
				# print(file_dir)
				with open(file_dir, "r") as f:
					content = f.read()
					# print(file_dir)
					content = resize_file_data(content)
					data.append(content)
					# Data and label type should be the same
					# Modify the labels to 0-35
					curr_label = int(ord(name))
					# print(f'the old label of {name} is: {curr_label}')
					if (curr_label > 57):
						curr_label = curr_label - 55
					else:
						curr_label = curr_label - 48
					label.append(curr_label)
					# print(f'the new label of {name} is: {curr_label}')

	# Shuffle the data and label in the same order
	idx = np.random.permutation(len(data))
	data = np.array(data)
	label = np.array(label)
	data_shuffled, label_shuffled = data[idx], label[idx]
	length = len(label_shuffled)
	# print(label)
	# print(label_shuffled)

	# Batch the data by BATCH_SIZE to format 4D tensor as input data
	dataset_len = int(length / BATCH_SIZE)
	# print(f'dataset_len = {dataset_len}')
	for count in range(dataset_len):
		this_data_batch = [None] * BATCH_SIZE
		this_label_batch = [None] * BATCH_SIZE
		for data_num in range(BATCH_SIZE):
			# print(f"data_num = {data_num}")
			data_index = count * BATCH_SIZE + data_num
			# print(f'data_index = {data_index}')
			this_data_batch[data_num] = data_shuffled[data_index]/ 128
			# this_data_batch[data_num] = this_data_batch[data_num].astype(int)
			this_label_batch[data_num] = label_shuffled[data_index]

		this_data_batch = tf.convert_to_tensor(this_data_batch, dtype=tf.float32)
		this_data_batch = tf.reshape(this_data_batch, [BATCH_SIZE, INPUT_HEIGHT, INPUT_WIDTH, INPUT_CHANNEL])
		my_dataset_data.append(this_data_batch)

		this_label_batch = tf.convert_to_tensor(this_label_batch, dtype=tf.int8)
		this_label_batch = tf.reshape(this_label_batch, [BATCH_SIZE])
		my_dataset_label.append(this_label_batch)

	print(data_type + "_total_data_length: " + str(length)) 
	return my_dataset_data, my_dataset_label, length

# Define a simple sequential model
def create_model():
	model = tf.keras.Sequential([
		layers.Conv2D(8, 5, padding='same', activation='relu', input_shape=(INPUT_HEIGHT, INPUT_WIDTH, INPUT_CHANNEL)),
		layers.MaxPooling2D((2, 2)),
		layers.Dropout(0.5),
		layers.Conv2D(16, 3, padding='same', activation='relu'),
		layers.MaxPooling2D((2, 2)),
		layers.Dropout(0.5),
		layers.Flatten(),
		layers.Dense(128, activation='relu'),
		# layers.Dropout(0.5),
		layers.Dense(NUM_CLASSES, activation="softmax")
	])

	model.compile(optimizer='adam',
		loss='sparse_categorical_crossentropy',
		metrics=['accuracy'])

	return model

if __name__ == "__main__":
	train_dir = './Users/xue_q/Downloads/keyboard_dataset/train'
	train_dir = pathlib.Path(train_dir)
	val_dir = './Users/xue_q/Downloads/keyboard_dataset/test'
	val_dir = pathlib.Path(val_dir)

	train_input_count = len(list(train_dir.glob('*/*.txt')))
	val_input_count = len(list(val_dir.glob('*/*.txt')))
	# print(f'{train_input_count}, {val_input_count}')

	class_names = np.array([item.name for item in train_dir.glob('*')])
	num_classes = len(class_names)
	if num_classes != NUM_CLASSES:
		print("Warning: Wrong classes number!")
		exit()
	# print(class_names)
	# print(num_classes)

	# Load the train and validation dataset
	train_ds, train_labels, train_len = file_dataset_from_directory(train_dir, "train")
	val_ds, val_labels, val_len = file_dataset_from_directory(val_dir, "validation")

	train_ds = tf.data.Dataset.from_tensor_slices((train_data, train_labels))
	val_ds = tf.data.Dataset.from_tensor_slices((val_data, val_labels))

	# Create a basic model instance
	model = create_model()
	model.summary()

	# Train the model with epochs
	epochs = 50
	# fit model
	his = model.fit(
		train_ds,
		validation_data = val_ds,
		epochs = epochs
	)

	# evaluate model
	_, acc = model.evaluate(val_ds, val_labels, verbose=0)
	print('> %.3f' % (acc * 100.0))

	plt.plot(his.history['accuracy'], label='acc', color='red')
	plt.plot(his.history['val_accuracy'], label='val_acc', color='green')
	plt.legend()

	# Save model
	model.save('weight.h5')

	# Convert the model to tflite model
	converter = tf.lite.TFLiteConverter.from_keras_model(model)
	tflite_model = converter.convert()

	# Save the tflite model
	with open('model.tflite', 'wb') as f:
		f.write(tflite_model)

	# Convert the tflite model to binary: use command xxd -i model.tflite > model_data.cc

