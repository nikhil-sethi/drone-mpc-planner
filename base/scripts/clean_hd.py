#!/usr/bin/env python3
import os
import shutil
import glob
import subprocess
import logging
from pathlib import Path
from datetime import datetime, timedelta
import lib_base as lb


def dir_to_datetime(dir_name):
    try:
        dir_date = datetime.strptime(os.path.basename(dir_name), "%Y%m%d_%H%M%S")
    except Exception:  # pylint: disable=broad-except
        dir_date = datetime(year=1, month=1, day=1)
    return dir_date


def disk_space(logger):
    total_used_space, _, free_space = shutil.disk_usage(lb.data_dir)
    logger.info('Free space: ' + str(free_space / 1024 / 1024 / 1024) + 'GB --> ' + str(round(free_space / total_used_space * 100)) + '% free')
    return free_space / total_used_space > 0.2


def strip_dir(dir_name: str, logger):
    logger.info('stripping: ' + dir_name)
    if os.path.isfile(dir_name + '/terminal.log'):
        os.remove(dir_name + '/terminal.log')
    if os.path.isfile(dir_name + '/log.csv'):
        os.remove(dir_name + '/log.csv')
    if os.path.isfile(dir_name + '/frames.csv'):
        os.remove(dir_name + '/frames.csv')
    vids = glob.glob(dir_name + "/*.mkv")
    vids.extend(glob.glob(dir_name + "/*.mp4"))
    for vid in vids:
        if Path(vid).stat().st_size > 100 * 1024 * 1024:  # 100 MB
            logger.info('removing: ' + vid)
            os.remove(vid)
    insect_logs = glob.glob(dir_name + '/log_itrk*.csv')
    for log in insect_logs:
        if Path(log).stat().st_size > 15 * 1024 * 1024:  # 15 MB
            logger.info('removing: ' + log)
            os.remove(log)
    dir_name = dir_name.replace('/logging', '')
    Path(dir_name + '/STRIPPED').touch()


def strip_folders_until(min_date, logger):
    check_free_space = True
    found_dirs = sorted(glob.glob(lb.data_dir + "*/202*_*"), key=dir_to_datetime)
    for dir_name in found_dirs:
        if check_free_space:
            if disk_space(logger):
                return True
        check_free_space = True
        try:
            dir_date = datetime.strptime(os.path.basename(dir_name), "%Y%m%d_%H%M%S")
        except Exception:  # pylint: disable=broad-except
            logger.error('could not get date from: ' + dir_name)
            logger.info('removing: ' + dir_name)
            shutil.rmtree(dir_name)
            continue
        if (min_date > dir_date) and not os.path.exists(dir_name + '/STRIPPED'):
            if os.path.exists(dir_name + '/junk'):
                logger.info('removing: ' + dir_name)
                shutil.rmtree(dir_name)
            else:
                strip_dir(dir_name, logger)
        elif os.path.exists(dir_name + '/STRIPPED'):
            check_free_space = False
        else:
            break
    return False


def remove_folders_until(min_date, logger):
    found_dirs = sorted(glob.glob(lb.data_dir + "*/202*_*"), key=dir_to_datetime)
    for dir_name in found_dirs:
        if disk_space(logger):
            return True
        try:
            dir_date = datetime.strptime(os.path.basename(dir_name), "%Y%m%d_%H%M%S")
        except Exception:  # pylint: disable=broad-except
            logger.error('could not get date from: ' + dir_name)
            logger.info('removing: ' + dir_name)
            shutil.rmtree(dir_name)
            continue
        if (min_date > dir_date) and os.path.exists(dir_name + '/STRIPPED'):
            logger.info('removing: ' + dir_name)
            shutil.rmtree(dir_name)
        else:
            break
    return False


def clean_hd():
    logger = logging.getLogger('clean_hd')

    if not os.path.exists(lb.data_dir):
        os.mkdir(lb.data_dir)

    result = subprocess.run('sudo logrotate /etc/logrotate.d/termlog_rotate_config', stdout=subprocess.PIPE, stderr=subprocess.PIPE, shell=True)
    if result.returncode != 0:
        logger.error(result.stdout.decode('utf-8'))
        logger.error(result.stderr.decode('utf-8'))
    if os.path.exists(lb.json_dir + 'sent/'):
        result = subprocess.run('find ' + lb.json_dir + 'sent/ -type f -mtime +30 -delete ', stdout=subprocess.PIPE, stderr=subprocess.PIPE, shell=True)
        if result.returncode != 0:
            logger.error(result.stdout.decode('utf-8'))
            logger.error(result.stderr.decode('utf-8'))

    if os.path.exists('/home/pats/pats/images/'):
        result = subprocess.run('find /home/pats/pats/images/ -type f -mtime +365 -delete ', stdout=subprocess.PIPE, stderr=subprocess.PIPE, shell=True)
        if result.returncode != 0:
            logger.error(result.stdout.decode('utf-8'))
            logger.error(result.stderr.decode('utf-8'))
    if os.path.exists(lb.trapeye_images_dir + 'sent/'):
        result = subprocess.run('find ' + lb.trapeye_images_dir + 'sent/ -type f -mtime +60 -delete ', stdout=subprocess.PIPE, stderr=subprocess.PIPE, shell=True)
        if result.returncode != 0:
            logger.error(result.stdout.decode('utf-8'))
            logger.error(result.stderr.decode('utf-8'))

    if disk_space(logger):
        return True

    if strip_folders_until(datetime.today() - timedelta(days=14), logger):
        return
    if remove_folders_until(datetime.today() - timedelta(days=14), logger):
        return
    try:
        logger.warning('Could not remove enough files older than two weeks')
    except OSError:  # OSError is called if we can not log because of full hd, so we catch that
        pass
    if strip_folders_until(datetime.today() - timedelta(days=1), logger):
        return
    try:
        logger.error('Could not strip enough files younger than two weeks')
    except OSError:  # OSError is called if we can not log because of full hd, so we catch that
        pass
    if remove_folders_until(datetime.today() - timedelta(days=1), logger):
        return
    try:
        logger.error('ONLY ONE DAY COULD BE KEPT!!!!!!!')
    except OSError:  # OSError is called if we can not log because of full hd, so we catch that
        pass


if __name__ == "__main__":
    clean_hd()
