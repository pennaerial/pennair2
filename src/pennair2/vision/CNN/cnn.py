from keras.models import Sequential
from keras import regularizers
from keras.preprocessing.image import ImageDataGenerator
from keras.layers import Convolution2D, MaxPooling2D
from keras.layers import Activation, Dropout, Flatten, Dense
import types
import numpy as np
import os
import tensorflow as tf

# extend keras model class
class CNN:

	def __init__(self,model=None,classes=2,weightspath="weights.h5",imgwidth=64,imgheight=64,imgdepth=1):
		self.imgwidth = imgwidth
		self.imgheight = imgheight
		self.imgdepth = imgdepth
		self.classes = classes
		self.model = model
		if model is None:
			self.classify_model()
		if weightspath is not None and os.path.exists(weightspath):
			self.model.load_weights(weightspath)


	def classify_model(self):

		model = Sequential()
		model.add(Convolution2D(8,7,7,input_shape=(self.imgheight,self.imgwidth,self.imgdepth),init='uniform'))
		model.add(Activation('relu'))
		model.add(MaxPooling2D(pool_size=(2,2)))

		model.add(Convolution2D(16,3,3,init='uniform'))
		model.add(Activation('relu'))
		model.add(MaxPooling2D(pool_size=(2,2)))

		model.add(Convolution2D(32,3,3,init='uniform'))
		model.add(Activation('relu'))
		model.add(MaxPooling2D(pool_size=(2,2)))

		model.add(Flatten())
		model.add(Dense(512))
		model.add(Activation('elu'))
		model.add(Dropout(0.25))

		model.add(Dense(len(self.classes)))
		model.add(Activation('elu'))

		model.compile(loss='categorical_crossentropy',optimizer='adam',metrics=['accuracy'])

		self.model = model


	def traingen(self,data,batchsize=32,zoom=0,rotation=0,shear=0,xshift=0,yshift=0,normalization=True):

		trainaugment = ImageDataGenerator(rescale=1./255,zoom_range=zoom,rotation_range=rotation,
										width_shift_range=xshift,height_shift_range=yshift,shear_range=shear,
										samplewise_std_normalization=normalization,samplewise_center=normalization)
		testaugment = ImageDataGenerator(rescale=1./255,
						samplewise_std_normalization=normalization,samplewise_center=normalization)

		colormode = ('grayscale' if self.imgdepth == 1 else 'rgb')
		if type(data) == tuple and type(data[0]) == str:
			# data = (img input dir, seg output dir, list of file paths txt)
			# format of VOC2012
			gen = target_size=(self.imgheight,self.imgwidth),
				data_dir=data[0],label_dir=data[1],file_path=data[2],
				class_mode='categorical',batch_size=batchsize,
				classes=self.classes,color_mode=colormode)
		if type(data) == str:
			# data = a directory with structure datadir/{y1,y2,...}/{x1,x2,x3,...}
			gen = trainaugment.flow_from_directory(data,
				target_size=(self.imgheight,self.imgwidth),
				class_mode='categorical',batch_size=batchsize,
				classes=self.classes,color_mode=colormode)
		else:
			# data = or a tuple of (x,y), where x and y are np arrays of the same length
			gen = trainaugment.flow(data[0],data[1],
				target_size=(self.imgheight,self.imgwidth),
				class_mode='categorical',batch_size=batchsize,
				classes=self.classes,color_mode=colormode)
		return gen


	def train(self,train,validate=None,numepoch=50,numtrain=2016,numtest=800):

		if isinstance(train,types.GeneratorType):
			if validate is not None:
				self.model.fit_generator(traingenerator,validation_data=valgenerator,
								nb_epoch=numepoch,samples_per_epoch=numtrain,nb_val_samples=numtest)
			else:
				self.model.fit_generator(traingenerator,nb_epoch=numepoch,samples_per_epoch=numtrain)

		self.model.save_weights(weightspath)


	# also handle if img is a list of images or a directory
	def predict(self,img):
		pass
