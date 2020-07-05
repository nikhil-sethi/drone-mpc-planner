#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from io import StringIO

def cleanWhitespaces(filepath):
	with open(filepath, "r") as file:
		datastring = file.read()

	datastring = datastring.replace(' ', '')
	datastring = StringIO(datastring)
	return datastring

if __name__ == "__main__":
	pass
