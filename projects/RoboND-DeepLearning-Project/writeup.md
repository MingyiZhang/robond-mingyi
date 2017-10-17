## Project: Follow Me

This project is aiming at design and train a convolutional neural network (ConvNet) such that a drone can recognize a particular person and follow him/her.

### Basic setup of the environment
The environment setup follows the instruction in the course and [here](https://github.com/udacity/RoboND-DeepLearning-Project).

Instead of using AWS, I use FloydHub to train the ConvNet. Because it is easy to setup and its gpu has more memories than AWS instance.

Simulator is downloaded from [here](https://github.com/udacity/RoboND-DeepLearning-Project/releases/tag/v1.2.2)

### Dataset Exploration
The dataset provided by the course contains three files:
- `train.zip`: training set,
    - `images/`: 4131 RGB images
    - `masks/`: 4131 mask images (Target: Blue; Pedestrian: Green; Background: Red)
- `validation.zip`: validation set,
    - `images/`: 1184 RGB images
    - `masks/`: 1184 mask images
- `sample_evaluation_data.zip`: test set, used for evaluation.
    - `following_images/`: image data that Target in the center.
        - `images/`: 542 RGB images
        - `masks/`: 542 mask images
    - `patrol_non_targ/`: image data without Target
        - `images/`: 270 RGB images
        - `masks/`: 270 mask images
    - `patrol_with_targ/`: image data with Target
        - `images/`: 322 RGB images
        - `masks/`: 322 mask images

I tried to collect images from the simulator, however it always crashes after taking 300 to 500 images... So in the training, I only use the provided dataset.

### Design of the ConvNet

The suggested architecture for this project is a Fully Convolutional Encoder-Decoder neural network.

The neural network contains no fully connected layers. It can be seperated into two main parts: encoder part and decoder part, constructed by encoder blocks and decoder blocks, respectively. The encoder block is defined as
```python
def encoder_block(input_layer, filters, strides):

    output_layer = SeparableConv2DKeras(filters=filters, kernel_size=3, strides=1,
                             padding='same', activation='relu')(input_layer)

    output_layer = separable_conv2d_batchnorm(output_layer, filters, strides)

    return output_layer
```
and the decoder block is defined as
```python
def decoder_block(small_ip_layer, large_ip_layer, filters):

    output = bilinear_upsample(small_ip_layer)

    output = layers.concatenate([output, large_ip_layer])

    output_layer = separable_conv2d_batchnorm(output, filters)

    return output_layer
```
In the encoder block, there are two separable convolutional layers (in order to emphasis and obtain more relevant patterns from the input images) and one batch normalization layer; while in the decoder block, there are one bilinear upsampling layer, one concatenate layer which merge two layers of the same spatial dimensions, one seperable convolutional layer and one batch normalization layer. The basic structure of my networks is a simplified U-net or [SegNet](http://blog.qure.ai/notes/semantic-segmentation-deep-learning-review).

After several attempts, I have two models, one with 3 encoder blocks and 3 decoder blocks,
```python
def fcn_model(inputs, num_classes):

    # Input
    # 256 x 256 x 3

    x1 = encoder_block(inputs, 32, 2)
    # 128 x 128 x 32
    x2 = encoder_block(x1, 64, 2)
    # 64 x 64 x 64
    x3 = encoder_block(x2, 128, 2)
    # 32 x 32 x 128

    x4 = conv2d_batchnorm(x3, 256, kernel_size=1, strides=1)
    # 32 x 32 x 256

    x5 = decoder_block(x4, x2, 128)
    # 64 x 64 x 128
    x6 = decoder_block(x5, x1, 64)
    # 128 x 128 x 64
    x = decoder_block(x6, inputs, 32)
    # 256 x 256 x 32

    return layers.Conv2D(num_classes, 3, activation='softmax', padding='same')(x)
```
 and another with 5 encoder blocks and 5 decoder blocks,
 ```python
 def fcn_model(inputs, num_classes):

     # Input
     # 256 x 256 x 3

     x1 = encoder_block(inputs, 8, 2)
     # 128 x 128 x 8
     x2 = encoder_block(x1, 16, 2)
     # 64 x 64 x 16
     x3 = encoder_block(x2, 32, 2)
     # 32 x 32 x 32
     x4 = encoder_block(x3, 64, 2)
     # 16 x 16 x 64
     x5 = encoder_block(x4, 128, 2)
     # 8 x 8 x 128

     x6 = conv2d_batchnorm(x5, 256, kernel_size=1, strides=1)
     # 8 x 8 x 256

     x7 = decoder_block(x6, x4, 128)
     # 16 x 16 x 128
     x8 = decoder_block(x7, x3, 64)
     # 32 x 32 x 64
     x9 = decoder_block(x8, x2, 32)
     # 64 x 64 x 32
     x10 = decoder_block(x9, x1, 16)
     # 128 x 128 x 16
     x = decoder_block(x10, inputs, 8)
     # 256 x 256 x 8

     return layers.Conv2D(num_classes, 3, activation='softmax', padding='same')(x)
 ```

### Model Training

I keep the input image size as 256 x 256 instead of 160 x 160.
The hyperparameter I choose are
```python
learning_rate = 0.01
batch_size = 32
num_epochs = 200
steps_per_epoch = 50
validation_steps = 37
workers = 2
```
With the learning rate as 0.01, models converge very fast, i.e. in 1 to 2 epochs. 32 as the batch size is standard. For the training step per epoch, I choose 50. In fact it can be smaller. The reason why I don't use the standard choice `steps_per_epoch = num_images / batch_size + 1` is two-fold.

First, in the training set generator `train_iter`, the argument `shuffle` is on by default. In fact, in file `code/utils/data_iterator.py`, the definition of class `BatchIteratorSimple(Iterator)` is initialized as `shuffle=True`
```python
class BatchIteratorSimple(Iterator):
    def __init__(self, data_folder, batch_size, image_shape,
            num_classes=3, training=True, shuffle=True, seed=None, shift_aug=False):
```
Which means that the data are shuffled between epochs.

Second, I uncomment the shift augmentation code in class `BatchIteratorSimple`
```python
if self.shift_aug:
    image, gt_image = shift_and_pad_augmentation(image, gt_image)
```
and set `shift_aug=True`, which introduces a random shift to the image and its mask.

Because of the above two reasons, there is hardly two image are the same among epochs. The `steps_per_epoch` can be whatever I want. Since I want to check the validation more often, a small `steps_per_epoch` is what I want.

`validation_steps` is taking the number `num_validation_images / batch_size + 1` because I want to check the `val_loss` for all validation images.

I use `ModelCheckpoint` callback to save the model with the lowest `val_loss`, and `EarlyStopping` callback to terminate the training if the model does not learning (However I didn't use it because I found there is no explicit seperation between training loss and validation loss), and the validation loss seems jumping among local minimals.

For the 3-encoder-3-decoder model, the best validation loss is 0.0181, while 0.14 for the 5-encoder-5-decoder model. (When training the 3-encoder-3-decoder, the floydhub instance stopped at epoch 127... but the best model is saved.)

### Evaluation
The final score are
- 3e3d: 0.482
- 5e5d: 0.505
In detail,
1. both model performs very well while the quad is following behind the target. They detect the target with very high accuracies.
2. models perform also good when there are no target visible. The rate of false positives is around 10% to 15%.
3. when target is far away, both models doesn't perform very well. The accuracy is around 50%... It might because the small amount of training set.

### Conclusions
This project shows how difficult the semantic segmentation is and how difficult to let a drone to follow a certain person automatically. Even with the simply environment setting (target is always in red) in this project, our model does not perform well enough when the target far away. 
