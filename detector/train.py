import numpy as np
import argparse
import os

from skimage.io import imsave, imread

from keras.callbacks import ModelCheckpoint

from helpers import print_heading
from data import load_train_data, load_test_data
from model import preprocess, get_unet



def train_and_predict(parent_folder):
    print_heading('Loading and preprocessing train data...')

    imgs_train, imgs_mask_train = load_train_data(parent_folder)

    imgs_train = preprocess(imgs_train)
    imgs_mask_train = preprocess(imgs_mask_train)

    imgs_train = imgs_train.astype('float32')

    mean = np.mean(imgs_train)  # mean for data centering
    std = np.std(imgs_train)  # std for data normalization

    if parent_folder == "carla":
        imgs_train -= mean
        imgs_train /= std

    imgs_mask_train = imgs_mask_train.astype('float32')
    imgs_mask_train /= 255.  # scale masks to [0, 1]

    print_heading('Creating and compiling model...')

    model = get_unet(parent_folder)
    
    model_checkpoint = ModelCheckpoint('tl_weights.h5', monitor='val_loss', save_best_only=True)

    print_heading('Fitting model...')

    model.fit(imgs_train, imgs_mask_train, batch_size=16, epochs=30, verbose=1, shuffle=True,
              validation_split=0.2,
              callbacks=[model_checkpoint])

    print_heading('Fitting model finished')
    
    print_heading('Loading and preprocessing test data...')

    imgs_test, imgs_id_test = load_test_data(parent_folder)
    imgs_test = preprocess(imgs_test)

    imgs_test = imgs_test.astype('float32')

    if parent_folder == "carla":
        imgs_test -= mean
        imgs_test /= std

    print_heading('Loading saved weights...')

    model.load_weights('tl_weights.h5')

    print_heading('Predicting masks on test data...')

    imgs_mask_test = model.predict(imgs_test, verbose=1)

    print_heading('Saving predicted masks to files...')

    pred_dir = 'preds'
    if not os.path.exists(pred_dir):
        os.mkdir(pred_dir)
    for image, image_id in zip(imgs_mask_test, imgs_id_test):
        image = (image[:, :, 0] * 255.).astype(np.uint8)
        imsave(os.path.join(pred_dir, image_id + '.pred.png'), image)

if __name__ == '__main__':
    parser = argparse.ArgumentParser()

    parser.add_argument(
        'parent_folder',
        type=str,
        nargs='?',
        default='carla',
        help='Path folder with mask images'
    )

    args = parser.parse_args()

    train_and_predict(args.parent_folder)