from __future__ import print_function

import os
import numpy as np
import argparse
import glob

from skimage.io import imsave, imread
from skimage import color
from skimage.filters import gaussian

from helpers import print_heading

data_folder = 'data'
temp_dir = 'temp'

image_rows = 600
image_cols = 800

def get_total_images(data_path):
    total = 0
    for folder in os.listdir(data_path):
        data_path_subfolder = os.path.join(data_path, folder, '*.jpg')
        images = glob.glob(data_path_subfolder)
        total += len(images)
    return total

def get_image_and_mask(image_name):
    image_mask_path = image_name.split('.')[0] + '_mask.jpg'
    img = imread(image_name)
    img_mask = np.array([])
    if os.path.exists(image_mask_path):
        img_mask = imread(image_mask_path)
    return np.array([img]), np.array([img_mask])

def change_brightness(image, image_name):
    image_hsv = color.rgb2hsv(image)
    random_bright = np.random.randint(2,8)/10
    value_layer = image_hsv[:,:,2]
    image_hsv[:,:,2] *= value_layer
    
    return (color.hsv2rgb(image_hsv)*255).astype('uint8')

def add_motion_blur(img, image_name):
    size = 3

    # generating the kernel
    kernel_motion_blur = np.zeros((size, size))
    kernel_motion_blur[int((size-1)/2), :] = np.ones(size)
    kernel_motion_blur = kernel_motion_blur / size

    return (gaussian(img, sigma=3, multichannel=True)*255).astype('uint8')

def create_train_data(folder, use_basic_augmentation):
    train_data_path = os.path.join(folder, data_folder, 'train')

    increment = 2 if use_basic_augmentation else 4
    total = get_total_images(train_data_path)*increment//2

    imgs = np.ndarray((total, image_rows, image_cols, 3), dtype=np.uint8)
    imgs_mask = np.ndarray((total, image_rows, image_cols, 3), dtype=np.uint8)

    print_heading('Creating training images...')

    i = 0
    for subfolder in os.listdir(train_data_path):
        train_data_path_subfolder = os.path.join(train_data_path, subfolder, '*.jpg')
        images = glob.glob(train_data_path_subfolder)
        for image_path in images:
            image_name = image_path.split('\\')[-1]
            if 'mask' in image_name:
                continue

            imgs[i], imgs_mask[i] = get_image_and_mask(image_path);

            imgs[i+1] = change_brightness(imgs[i], image_name)
            imgs_mask[i+1] = imgs_mask[i]

            imsave(os.path.join(temp_dir, 'brightness_' + image_name), imgs[i+1])

            if not use_basic_augmentation:
                imgs[i+2] = add_motion_blur(imgs[i], image_name)
                imgs_mask[i+2] = imgs_mask[i]

                imsave(os.path.join(temp_dir, 'blur_' + image_name), imgs[i+2])

                imgs[i+3] = add_motion_blur(imgs[i+1], image_name)
                imgs_mask[i+3] = imgs_mask[i]

                imsave(os.path.join(temp_dir, 'both_' + image_name), imgs[i+3])

            if i % 10 == 0:
                print('Done: {0}/{1} images'.format(i, total))
            i += increment

    print_heading('Loading done.')

    np.save(os.path.join(folder, 'tl_train.npy'), imgs)
    np.save(os.path.join(folder, 'tl_mask_train.npy'), imgs_mask)

    print_heading('Saving to .npy files done.')


def load_train_data(folder):
    imgs_train = np.load(os.path.join(folder, 'tl_train.npy'))
    imgs_mask_train = np.load(os.path.join(folder, 'tl_mask_train.npy'))
    return imgs_train, imgs_mask_train


def create_test_data(folder):
    test_data_path = os.path.join(folder, data_folder, 'test')
    total = get_total_images(test_data_path)

    imgs = np.ndarray((total, image_rows, image_cols, 3), dtype=np.uint8)
    imgs_id = np.ndarray((total, ), dtype=object)

    i = 0
    print_heading('Creating test images...')

    for subfolder in os.listdir(test_data_path):
        test_data_path_subfolder = os.path.join(test_data_path, subfolder, '*.jpg')
        images = glob.glob(test_data_path_subfolder)
        for image_path in images:
            image_name = image_path.split('\\')[-1]
            if 'mask' in image_name:
                continue

            imgs[i], _ = get_image_and_mask(image_path);
            imgs_id[i] = image_name.split('.')[0]

            if i % 10 == 0:
                print('Done: {0}/{1} images'.format(i, total))
            i += 1

    print_heading('Loading done.')

    np.save(os.path.join(folder, 'tl_test.npy'), imgs)
    np.save(os.path.join(folder, 'tl_id_test.npy'), imgs_id)

    print_heading('Saving to .npy files done.')


def load_test_data(folder):
    imgs_test = np.load(os.path.join(folder, 'tl_test.npy'))
    imgs_mask_test = np.load(os.path.join(folder, 'tl_id_test.npy'))
    return imgs_test, imgs_mask_test

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

    if not os.path.exists(temp_dir):
        os.mkdir(temp_dir)

    create_train_data(args.parent_folder, args.parent_folder != 'carla')
    create_test_data(args.parent_folder)
