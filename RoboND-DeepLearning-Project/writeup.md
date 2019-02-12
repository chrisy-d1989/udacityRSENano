## Project: FollowMe

[//]: # (Image References)

[image1]: ./docs/pictures/sim_follow.png
[image2]: ./docs/pictures/fcn.png
[image3]: ./docs/pictures/en3_lr0001.png
[image4]: ./docs/pictures/pic_3en_200spe_15ep.png

##Contents
* Introduction
* Neural Network
* Training and Validation
* Improvements

## Introduction

This porject is about deep learning to perform sematic segmentation. The fully convolutional network (fcn) has to recognize a specific person (hero) with a red dress in a quadcopter simulation environment. Within this simulation the training, validation and test data for the fcn is generated and processed. The goal of the project is to building and train a fcn that classifies each pixel of the camera into non-human, human and hero, therefore the quadcopter can follow the hero after finding it. Figure 1 shows the quadcopter while following the hero.  

![alt text][image1]
*Figure 1 - Simulation Environment with Hero and Quadcopter*

## Fully Convolutional Network

Figure 2 shows one of the fcn's I implemented for training. It consists of:

* Input layer as the image from the quadcopter's camera.
* 3 encoder layers which each consists of a seperable 2d convolution with a ReLu activation function, same padding a specific filter size and strides of 2 and a following batch normalization. This encoder layers helps to skip connections between the layers which allows the network th use information from multiple resolutions. This helps to make better segmentation decisions and extract specific features.
* 1x1 deep convolutional layer, that helps to make the model more deeper and give it more parameters.
* 3 decoder layers help to upsample the network layers to the size of the input layer and skip connections from the input and encoder layers for each decoder layer. 
* An output layer with a softmax activation function and a same padding.

I decied to build a fcn because the lessons mentioned that fnc's do well on sematic segmentation tasks in contrast to convolutional neural networks which wouldn't have spatial information and therefore are unable to perform well on semantic segmentation. This is maily because of the 1x1 deep convolutional layer, which reduces the input dimensionality but doesn't eliminate the spatial information. Furthermore, it allows the decoder layers to perform upsampling.

![alt text][image2]
*Figure 3 - Example of Fully Convolutional Network*

##Training and Validation

I created the design of the fcn in figure 2 with trial and error of different fnc's designs. I also tried to reduce the size of the fcn by trhowing out layer 3 (and 4) respectively 6 (and 7) while training with the data sets provided by Udacity. However I got the best results with the design of three encoders and decoders. Therefore, I tried to tune the hyperparameters with this fcn as can be seen in the table below. I evaluated every fcn and hyperparamter setup with the training and validation loss as well es the evaluation scores provided in the notebook.

NoE | Filter | S | Weight | IoU | Score | LR | BS | NE | SE | VS | W | Val Loss | Loss | Worked |
--- | --- | --- | --- | --- | --- | --- | --- | --- --- | --- | --- | --- | --- | --- | 
1 | 32 | 2 | 0.70195 | 0.0766| 0.0538 | 0.01 | 26 | 30 | 300 | 50 | 2 | 0.0622 | 0.0481 | yes |
2 |  32/64 | 2 | 0.7237 | 0.52725| 0.3815 | 0.01 | 26 | 30 | 300 | 50 | 2 | 0.0317 | 0.0246 | yes |
3 | 32/64/128 | 2 | 0.7290 | 0.5363| 0.3909 | 0.01 | 26 | 30 | 300 | 50 | 2 | 0.0239 | 0.0165 | yes |
3 | 32/64/128 | 2 | 0.7290 | 0.5503| 0.4011 | 0.01 | 26 | 15 | 300 | 50 | 2 | 0.0332| 0.0165 | yes |
3 | 32/64/128 | 2 | 0.7254 | 0.5473| 0.3970 | 0.01 | 26 | 15 | 200 | 50 | 2 | 0.0316| 0.0121| yes |
3 | 32/64/128 | 2 | 0.7412 | 0.5432| 0.4026 | 0.1 | 26 | 30 | 300 | 50 | 2 | 0.0233| 0.0131| yes |
3 | 32/64/128 | 2 | 0.7162 | 0.5224| 0.3741 | 0.001 | 26 | 30 | 300 | 50 | 2 | 0.0323| 0.015 | yes |
3 | 32/64/128 | 2 | 0.6454 | 0.3817| 0.2464 | 0.0001 | 26 | 10 | 300 | 50 | 2 | 0.0685|0.0643 | yes |

NoE = number of encoder layers (equals number of decoder layers)
S = Stride
LR = learning rate
BS = batch size
NS = number of epochs
SE = steps per Epoch
VS = validation step
W = workers
worked = if the net successfully found an followed the hero

The learning rates where choosen by trial and error as well as the epoch numbers. I got good results with a learning rate of 0.01 and the epoch number of 30. I also tried a learning rate of 0.01 and an epoch number of 30. In this case the validation and training loss were almost similiar for the last 15 epochs. Therefore, you can get a faster success with a learning rate of 0.001. Figure 3 shows the final result of the test setup with three encoders and decoders and the hyperparameters from row 4 of the table. 

![alt text][image3]
*Figure 3 - Final Results of Training*

The notebook evaluated the performance of the fcn's with a few examples shown in figure 4. The raw image is on the left, the groundtruth data in the middle and the predicted labels on the right side of the image. 

Nevertheless, every fcn shown in the table above was able to solve the task of recognizing the hero and following it.

![alt text][image4]
*Figure 4 - Evaluation Example*

## Improvements

The network doesn't classify anything beyond non-human, human and hero. To identify cats, dogs and cars, the fcns have to be trained with data which clearly labels these different classes. 