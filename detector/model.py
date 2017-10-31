import numpy as np
import os

from skimage.transform import resize
from skimage.color import rgb2grey

from keras.models import Model
from keras.layers import Input, concatenate, Conv2D, MaxPooling2D, Conv2DTranspose
from keras.optimizers import Adam
from keras import backend as K


img_rows = 96
img_cols = 128

smooth = 1.

def dice_coef(y_true, y_pred):
    y_true_f = K.flatten(y_true)
    y_pred_f = K.flatten(y_pred)
    intersection = K.sum(y_true_f * y_pred_f)
    return (2. * intersection + smooth) / (K.sum(y_true_f) + K.sum(y_pred_f) + smooth)

def dice_coef_loss(y_true, y_pred):
    return -dice_coef(y_true, y_pred)

def get_unet(parent_folder):
    inputs = Input((img_rows, img_cols, 1))
    conv1 = Conv2D(32, (3, 3), activation='relu', padding='same', trainable=True)(inputs)
    conv1 = Conv2D(32, (3, 3), activation='relu', padding='same', name='conv_1_2', trainable=True)(conv1)
    pool1 = MaxPooling2D(pool_size=(2, 2), name='maxpool_1', trainable=True)(conv1)

    conv2 = Conv2D(64, (3, 3), activation='relu', padding='same', name='conv_2_1', trainable=True)(pool1)
    conv2 = Conv2D(64, (3, 3), activation='relu', padding='same', name='conv_2_2', trainable=True)(conv2)
    pool2 = MaxPooling2D(pool_size=(2, 2), name='maxpool_2', trainable=True)(conv2)

    conv3 = Conv2D(128, (3, 3), activation='relu', padding='same', name='conv_3_1', trainable=True)(pool2)
    conv3 = Conv2D(128, (3, 3), activation='relu', padding='same', name='conv_3_2', trainable=True)(conv3)
    pool3 = MaxPooling2D(pool_size=(2, 2), name='maxpool_3', trainable=True)(conv3)

    conv4 = Conv2D(256, (3, 3), activation='relu', padding='same', name='conv_4_1', trainable=True)(pool3)
    conv4 = Conv2D(256, (3, 3), activation='relu', padding='same', name='conv_4_2', trainable=True)(conv4)
    pool4 = MaxPooling2D(pool_size=(2, 2), name='maxpool_4', trainable=True)(conv4)

    conv5 = Conv2D(512, (3, 3), activation='relu', padding='same', name='conv_5_1', trainable=True)(pool4)
    conv5 = Conv2D(512, (3, 3), activation='relu', padding='same', name='conv_5_2', trainable=True)(conv5)

    up6 = concatenate([Conv2DTranspose(256, (2, 2), strides=(2, 2), padding='same', name='convtran_6', trainable=True)(conv5), conv4], name='up_6', trainable=True, axis=3)
    conv6 = Conv2D(256, (3, 3), activation='relu', padding='same', name='conv_6_1', trainable=True)(up6)
    conv6 = Conv2D(256, (3, 3), activation='relu', padding='same', name='conv_6_2', trainable=True)(conv6)

    up7 = concatenate([Conv2DTranspose(128, (2, 2), strides=(2, 2), padding='same', name='convtran_7', trainable=True)(conv6), conv3], name='up_7', trainable=True, axis=3)
    conv7 = Conv2D(128, (3, 3), activation='relu', padding='same', name='conv_7_1', trainable=True)(up7)
    conv7 = Conv2D(128, (3, 3), activation='relu', padding='same', name='conv_7_2', trainable=True)(conv7)

    up8 = concatenate([Conv2DTranspose(64, (2, 2), strides=(2, 2), padding='same', name='convtran_8', trainable=True)(conv7), conv2], name='up_8', trainable=True, axis=3)
    conv8 = Conv2D(64, (3, 3), activation='relu', padding='same', name='conv_8_1', trainable=True)(up8)
    conv8 = Conv2D(64, (3, 3), activation='relu', padding='same', name='conv_8_2', trainable=True)(conv8)

    up9 = concatenate([Conv2DTranspose(32, (2, 2), strides=(2, 2), padding='same', name='convtran_9', trainable=True)(conv8), conv1], name='up_9', trainable=True, axis=3)
    conv9 = Conv2D(32, (3, 3), activation='relu', padding='same', name='conv_9_1', trainable=True)(up9)
    conv9 = Conv2D(32, (3, 3), activation='relu', padding='same', name='conv_9_2', trainable=True)(conv9)

    conv10 = Conv2D(1, (1, 1), activation='sigmoid')(conv9)

    model = Model(inputs=[inputs], outputs=[conv10])

    model.load_weights(os.path.join(parent_folder, 'weights.h5'), by_name=True)
    #model.load_weights('tl_weights.h5', by_name=True)

    model.compile(optimizer=Adam(lr=1e-5), loss=dice_coef_loss, metrics=[dice_coef])

    model.summary()

    return model

def preprocess(imgs):
    imgs_p = np.ndarray((imgs.shape[0], img_rows, img_cols), dtype=np.uint8)
    for i in range(imgs.shape[0]):
        imgs_p[i] = rgb2grey(resize(imgs[i], (img_rows, img_cols, 3), preserve_range=True, mode="constant"))

    imgs_p = imgs_p[..., np.newaxis]
    return imgs_p
