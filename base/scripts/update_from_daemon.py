#!/usr/bin/env python3
import os
import subprocess
import socket
import logging
import lib_base as lb


def touch(fn: str):
    with open(fn, 'w', encoding="utf-8") as file:
        file.write('')


def untouch(fn: str):
    if os.path.exists(fn):
        os.remove(fn)


def update(logger_name):
    logger = logging.getLogger(logger_name)

    if os.path.exists(lb.disable_updates):
        return
    pats_id = socket.gethostname()
    if 'pats' not in pats_id:
        return

    pats_settings_currently = {}
    if os.path.exists(lb.pats_xml_fn):
        pats_settings_currently = lb.read_xml(lb.pats_xml_fn)

    rsync_cmd = 'rsync -aLz dash_upload:xml/' + pats_id + '.xml ' + lb.pats_xml_fn
    result = subprocess.run(rsync_cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, shell=True)
    if result.returncode != 0:
        logger.error(result.stdout)
        logger.error(result.stderr)
        return

    pats_settings_new = lb.read_xml(lb.pats_xml_fn)
    if pats_settings_currently == pats_settings_new:
        return
    logger.info("Downloaded changed pats.xml")

    if 'tag' not in pats_settings_new:
        return
    if 'tag' not in pats_settings_currently or pats_settings_new['tag'] != pats_settings_currently['tag']:
        logger.info("Updating to: " + pats_settings_new['tag'])
        cmd = 'cd ~/pats/release && git fetch --tags && git co ' + pats_settings_new['tag'] + ' && kill $(pgrep -f baseboardlink.py) && kill $(pgrep -f endless_wait.sh)'
        result = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, shell=True)
        if result.returncode != 0:
            logger.error(result.stdout)
            logger.error(result.stderr)

    if pats_settings_new['op_mode'] != pats_settings_currently['op_mode']:
        logger.info("Changing mode to: " + pats_settings_new['op_mode'])

        if pats_settings_new['op_mode'] == 'op_mode_disabled':
            touch(lb.disable_flag)

        if pats_settings_new['op_mode'] == 'op_mode_blind':
            untouch(lb.disable_flag)
            touch(lb.disable_executor_flag)

        if pats_settings_new['op_mode'] == 'op_mode_c':
            untouch(lb.disable_flag)
            untouch(lb.disable_executor_flag)

        if pats_settings_new['op_mode'] == 'op_mode_x':
            untouch(lb.disable_flag)
            untouch(lb.disable_executor_flag)

    if 'trapeye' not in pats_settings_currently or pats_settings_new['trapeye'] != pats_settings_currently['trapeye']:
        # todo, call configure trapeye script
        if pats_settings_new['trapeye']:
            logger.info("Enabling trap-eye")
        else:
            logger.info("Disabling trap-eye")

    if 'tag' not in pats_settings_currently or pats_settings_new['tag'] != pats_settings_currently['tag']:
        cmd = 'kill $(pgrep -f daemon.py)'
        result = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, shell=True)


if __name__ == "__main__":
    update('update_from_daemon')
