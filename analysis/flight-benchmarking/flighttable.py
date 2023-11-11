#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import numpy as np

from flightstates import flightstates_evaldata_units
from terminallog import terminal_evaldata_units, log_data_units
from control import control_evaldata_units
from longrangeflight import longrange_evaldata_units
from hunt import hunt_evaldata_units


class Flighttable(object):
    def __init__(self):
        self.__seperator = '\t'
        self.tablecontent = 'Date' + self.__seperator + 'Time' + self.__seperator + 'System' + self.__seperator + 'Logname'

        self.f_edatunits = flightstates_evaldata_units()
        for key in self.f_edatunits:
            self.tablecontent += self.__seperator + key + ' [' + str(self.f_edatunits[key]) + ']'

        self.t_edatunits = terminal_evaldata_units()
        for key in self.t_edatunits:
            self.tablecontent += self.__seperator + key + ' [' + str(self.t_edatunits[key]) + ']'

        self.c_edatunits = control_evaldata_units()
        for key in self.c_edatunits:
            self.tablecontent += self.__seperator + key + ' [' + self.c_edatunits[key] + ']'

        self.l_edatunits = longrange_evaldata_units()
        for key in self.l_edatunits:
            self.tablecontent += self.__seperator + key + ' [' + self.l_edatunits[key] + ']'

        self.h_edatunits = hunt_evaldata_units()
        for key in self.h_edatunits:
            self.tablecontent += self.__seperator + key + ' [' + self.h_edatunits[key] + ']'

        self.tablecontent += '\n'

    def set_dat2line(self, data, data_units):
        first = True
        for key in data_units:
            if first:
                first = False
            else:
                self.tablecontent += self.__seperator

            if key in data:
                if isinstance(data[key], list):
                    self.tablecontent += '{:.3f}'.format(np.max(data[key]))
                elif isinstance(data[key], (str, bool)):
                    self.tablecontent += str(data[key])
                else:
                    self.tablecontent += '{:.3f}'.format(data[key])
            else:
                pass

    def add(self, log_dat, step_edat, fs_edat, ter_edat, hunt_edat, lr_edat):
        self.set_dat2line(log_dat, log_data_units())
        self.tablecontent += self.__seperator
        self.set_dat2line(fs_edat, self.f_edatunits)
        self.tablecontent += self.__seperator
        self.set_dat2line(ter_edat, self.t_edatunits)
        self.tablecontent += self.__seperator
        self.set_dat2line(step_edat, self.c_edatunits)
        self.tablecontent += self.__seperator
        self.set_dat2line(lr_edat, self.l_edatunits)
        self.tablecontent += self.__seperator
        self.set_dat2line(hunt_edat, self.h_edatunits)

        self.tablecontent += '\n'


def gen_tabletemplate():
    return "Date\tTime\tSystem"


def append_newflight(table, flight_data):
    pass


if __name__ == "__main__":
    flighttable = Flighttable()
    print(flighttable.tablecontent)
