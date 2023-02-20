#!/usr/bin/env python3
import lib_base as lb
import logging
import pickle
import glob
import os
from typing import List


def save_database(database):
    with open(lb.trapeye_pkl, 'wb') as f:
        pickle.dump(database, f)


def load_database():
    with open(lb.trapeye_pkl, 'rb') as f:
        return pickle.load(f)


def apply_ids_and_move_for_send(image_paths: List[str]):
    trap_ids = load_database()
    for im in image_paths:
        node_id = int(os.path.basename(im).split('_')[3])
        trap_nr = 0
        if node_id in trap_ids:
            trap_nr = int(trap_ids[node_id])
        new_name_parts = os.path.basename(im).split('_')
        new_name_parts[2] = "trap{:03}".format(trap_nr)
        new_im = lb.trapeye_images_dir + 'to_send/' + "_".join(new_name_parts)
        os.rename(im, new_im)


def upload_images():
    logger = logging.getLogger('trapeye')

    image_paths = lb.natural_sort(glob.glob(os.path.expanduser(lb.trapeye_images_dir) + '/*.jpg'))
    image_paths.reverse()
    if not os.path.exists(lb.trapeye_images_dir + 'to_send'):
        os.mkdir(lb.trapeye_images_dir + 'to_send')
    if not os.path.exists(lb.trapeye_images_dir + 'sent'):
        os.mkdir(lb.trapeye_images_dir + 'sent')
    apply_ids_and_move_for_send(image_paths)

    cmd = ' rsync -a ' + lb.trapeye_images_dir + 'to_send/*' + ' dash:trapeye_images/'

    send_result = lb.execute(cmd, 3, 'trapeye')
    if not send_result:
        lb.execute('mv ' + lb.trapeye_images_dir + 'to_send/* ' + lb.trapeye_images_dir + 'sent/', 1, 'trapeye')

    return send_result


if __name__ == "__main__":
    logging.basicConfig()
    logger = logging.getLogger('trapeye')
    logger.setLevel(logging.DEBUG)

    upload_images()
