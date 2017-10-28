import numpy as np
import os
import argparse

from skimage.io import imsave, imread
from skimage.transform import resize

from helpers import print_heading

from data import load_test_data
from model import img_rows, img_cols

image_rows = 600
image_cols = 800

row_ratio = image_rows/img_rows
col_ratio = image_cols/img_cols

middle_row = img_rows / 2
middle_col = img_cols / 2

root_path = os.path.dirname(os.path.realpath(__file__))

def preprocess(img):
    return resize(img, (img_rows, img_cols), preserve_range=True, mode="constant").astype('float32') / 255;

def extract_image(pred_image_mask, image):
    column_projection = np.sum(pred_image_mask, axis = 0)

    if (np.max(column_projection) < 20*20):
        return None

    non_zero_column_index = np.argwhere(column_projection > 20*20)

    if non_zero_column_index.size == 0:
        return None

    index_of_column_index =  np.argmin(np.abs(non_zero_column_index - middle_col))
    column_index = non_zero_column_index[index_of_column_index][0]

    zero_colum_indexes = np.argwhere(column_projection == 0)
    left = np.max(zero_colum_indexes[zero_colum_indexes < column_index])
    right = np.min(zero_colum_indexes[zero_colum_indexes > column_index])

    #roi = pred_image_mask[:,left:right]

    row_projection = np.sum(pred_image_mask, axis = 1)
    row_index =  np.argmax(row_projection)

    zero_row_indexes = np.argwhere(row_projection == 0)
    top = np.max(zero_row_indexes[zero_row_indexes < row_index])
    bottom = np.min(zero_row_indexes[zero_row_indexes > row_index])

    return image[int(top*row_ratio):int(bottom*row_ratio), int(col_ratio*left):int(col_ratio*right)]

def apply_mask(mode):

    print_heading('Apply mask to test images ...')

    pred_dir = os.path.join(root_path, 'preds')
    imgs_test, imgs_id_test = load_test_data(os.path.join(root_path, mode))

    for image, image_id in zip(imgs_test, imgs_id_test):
        pred_image_name = os.path.join(pred_dir, str(image_id) + '.pred.png')
        pred_image_mask = imread(pred_image_name)

        extracted = extract_image(pred_image_mask, image)
        if extracted is None:
            print("miss", image_id)
        else:
            imsave(os.path.join(pred_dir, image_id + '.result.jpg'), extracted)

def apply_mask_custom(folder):

    print_heading('Apply mask to test images ...')

    pred_dir = os.path.join(folder, 'preds')
    if not os.path.exists(pred_dir):
        os.mkdir(pred_dir)
    print(pred_dir)

    for image_name in os.listdir(pred_dir):
        image_path = os.path.join(folder, image_name.split('.')[0] + '.jpg')
        image = imread(image_path)

        pred_image_name = os.path.join(pred_dir, image_name)
        pred_image_mask = imread(pred_image_name)

        extracted = extract_image(pred_image_mask, image)
        if extracted is None:
            print("miss", image_name)
        else:
            imsave(os.path.join(pred_dir, image_name.split('.')[0] + '.result.jpg'), extracted)

if __name__ == '__main__':

    parser = argparse.ArgumentParser()
    parser.add_argument(
        'folder',
        type=str,
        help='Path folder with mask images'
    )
    parser.add_argument(
        'mode', # carla, sim
        type=str,
        nargs='?',
        default='none',
        help='Path folder with mask images'
    )
    args = parser.parse_args()

    if args.mode == 'none':
        apply_mask_custom(args.folder)
    else:
        apply_mask(args.mode)
    