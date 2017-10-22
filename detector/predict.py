import os
import numpy as np
import argparse
import glob

from skimage.io import imsave, imread
import tensorflow as tf

from helpers import print_heading
from model import preprocess, get_unet

miss = 100

def predict(folder_name, mode):
    print_heading('Loading and preprocessing data...')

    image_rows = 600
    image_cols = 800
    folder_pattern = os.path.join(folder_name, '*.jpg')
    image_list = glob.glob(folder_pattern)

    total = int(len(image_list)/miss) + 1 if miss > 0 else len(image_list)
    imgs = np.ndarray((total, image_rows, image_cols, 3), dtype=np.uint8)
    imgs_id = np.ndarray((total,), dtype=object)

    count = 0
    for image_path in image_list:
        image_name = image_path.split('\\')[-1]
        if miss == 0 or count % miss == 1:
            index = int(count / miss) if miss > 0 else count
            imgs[index] = np.array([imread(os.path.join(folder_name, image_name))])
            imgs_id[index] = image_name.split('.')[0]
        count += 1

    imgs = preprocess(imgs).astype('float32')

    if mode == "carla":
        mean = np.mean(imgs)  # mean for data centering
        std = np.std(imgs)  # std for data normalization
    
        imgs -= mean
        imgs /= std

    print_heading('Loading saved weights...')

    model = get_unet(mode)
    model.load_weights(os.path.join(mode, 'tl_weights.h5'))

    print_heading('Predicting masks on test data...')

    imgs_mask = model.predict(imgs, verbose=1)

    print_heading('Saving predicted masks to files...')

    pred_dir = os.path.join(folder_name, 'preds')
    if not os.path.exists(pred_dir):
        os.mkdir(pred_dir)
    for image, image_id in zip(imgs_mask, imgs_id):
        image = (image[:, :, 0] * 255.).astype(np.uint8)
        imsave(os.path.join(pred_dir, image_id + '.pred.png'), image)

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument(
        'folder',
        type=str,
        help='Path folder with images to predict'
    )
    parser.add_argument(
        'mode', # carla or sim
        type=str,
        help='Path folder with images to predict'
    )
    args = parser.parse_args()

    predict(args.folder, args.mode)