#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import numpy as np

KEY_TAKEOFFTIME = 'takeoff_time'
KEY_LANDEDTIME = 'landed_time'
KET_ENTERLANDINGTIME = 'entering_landing_time'
KEY_CRASHED = 'crashed'

NS_TAKING_OFF = 10
NS_LANDING = 23
NS_LANDED = 24


def init_flightstate_data():
	return {KEY_TAKEOFFTIME: np.nan, KEY_LANDEDTIME: np.nan, KET_ENTERLANDINGTIME: np.nan}

def eval_current_flightstate(flight, time, nav_state, auto_throttle):
	if nav_state == NS_LANDING and np.isnan(flight[KET_ENTERLANDINGTIME]):
		flight[KET_ENTERLANDINGTIME] = time

	if nav_state == NS_LANDED and np.isnan(flight[KEY_LANDEDTIME]):
		flight[KEY_LANDEDTIME] = time

	if nav_state == NS_TAKING_OFF and np.isnan(flight[KEY_TAKEOFFTIME]):
		flight[KEY_TAKEOFFTIME] = time
	return flight

KEY_MISSIONTIME = 'mission_time'
KEY_LANDINGTIME = 'landing_time'
def flightstates_evaldata(flightstates_data, crashed_after_landing):
	flightstates_evaldata = {}

	if flightstates_data:
		flight = flightstates_data
		flightstates_evaldata[KEY_CRASHED] = False
		if flight:
			if crashed_after_landing or np.isnan(flight[KEY_LANDEDTIME]):
				flightstates_evaldata[KEY_CRASHED] = True
			if not np.isnan(flight[KEY_TAKEOFFTIME]) and not np.isnan(flight[KET_ENTERLANDINGTIME]):
				flightstates_evaldata[KEY_MISSIONTIME] = flight[KET_ENTERLANDINGTIME] - flight[KEY_TAKEOFFTIME]
			if not np.isnan(flight[KET_ENTERLANDINGTIME]) and not np.isnan(flight[KEY_LANDEDTIME]):
				flightstates_evaldata[KEY_LANDINGTIME] = flight[KEY_LANDEDTIME] - flight[KET_ENTERLANDINGTIME]

	return flightstates_evaldata

def flightstates_evaldata_units():
	flightstates_evaldata_unit = {}
	flightstates_evaldata_unit[KEY_CRASHED] = 1
	flightstates_evaldata_unit[KEY_MISSIONTIME] = 's'
	flightstates_evaldata_unit[KEY_LANDINGTIME] = 's'

	return flightstates_evaldata_unit

if __name__ == "__main__":
	pass
