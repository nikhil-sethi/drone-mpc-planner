#!/usr/bin/env python3
# -*- coding: utf-8 -*-

def extract_variables_from_string_list(text):
    text = text.replace('[','')
    text = text.replace(']','')
    text = text.replace(',','')
    text = text.split(' ')
    if text[0] == '': # List is empty
        text = []
    return text

def generalize_formulation(formulation):
    """Get rid of specific variable and parameter names."""
    formulation_lines = formulation.split('\n')
    xopt = extract_variables_from_string_list(formulation_lines[1])
    params = extract_variables_from_string_list(formulation_lines[3])
    lamg = extract_variables_from_string_list(formulation_lines[5])

    nx = len(xopt)
    for xidx, x in enumerate(list(reversed(xopt))):
        formulation = formulation.replace(x, "xopt"+str(nx - xidx - 1))

    np = len(params)
    for pidx, p in enumerate(list(reversed(params))):
        formulation = formulation.replace(p, "param"+str(np - pidx - 1))

    nl = len(lamg)
    for lidx, l in enumerate(list(reversed(lamg))):
        formulation = formulation.replace(l, "lamg"+str(nl - lidx - 1))

    return formulation

def find_nxopts(formulation_lines):
    xopt_line = formulation_lines[1]
    last_xopt = xopt_line.split(", ")[-1]
    last_xopt_idx = last_xopt.replace("xopt", "").replace("]", "")

    return int(last_xopt_idx) + 1
