#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from io import StringIO

def cleanWhitespaces(filepath):
	file = open(filepath, "r")
	datastring = file.read()
	file.close()
	datastring = datastring.replace(' ', '')
	datastring = StringIO(datastring)
	return datastring

if __name__ == "__main__":
	pass