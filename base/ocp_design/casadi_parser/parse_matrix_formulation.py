#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import re #library for regexp

def is_definition(line):
    """."""
    if line[:1] == "@":
        tmp = line.split('=')
        if(len(tmp)>1):
            return True
    return False

def get_definition(line):
    if line[:1] == "@":
        tmp = line.split('=')
        return tmp[0], tmp[1].replace(",", "")

def find_definitions(lines):
    defs = {}
    mark_to_delete = []
    for line, line_content in enumerate(lines):
        line_content = line_content.replace(" ", "")
        if is_definition(line_content):
            key, expression = get_definition(line_content)
            defs[key] = expression
            mark_to_delete.append(line)

    for line_idx in list(reversed(mark_to_delete)):
        del lines[line_idx]

    return defs, lines

def insert_definitions(consts, lines):
    for line, line_content in enumerate(lines):
        lines[line] = lines[line].replace(" ", "")
        for const in consts:
            lines[line] = lines[line].replace(const, consts[const])

    for line, line_content in enumerate(lines):
        if line_content == "":
            del lines[line]

    return lines

def parse_sparse_matrix_entry(line):
    """Expects lines like: (0, 46) -> 0.001."""
    tmp = line.split("->")

    tmp_idx = tmp[0].split(",")
    row = int(tmp_idx[0].replace("(", ""))
    colum = int(tmp_idx[1].replace(")", ""))
    expr = tmp[1]
    return [row, colum, parse_expression(expr)]

def parse_matrix_formulation(lines):
    """Return a list of [row, column, expression]."""
    defs, lines = find_definitions(lines)
    lines = insert_definitions(defs, lines)

    parsed_matrix = []
    for line in lines:
        parsed_matrix.append(parse_sparse_matrix_entry(line))

    return parsed_matrix

def parse_complex_matrix_formulation(lines):
    """Similar to parse_matrix_formulation() but also returning defs."""
    defs, lines = find_definitions(lines)

    lines[0] = lines[0].replace("[", "")
    lines[-1] = lines[-1].replace("]", "")

    parsed_matrix = []
    for line in lines:
        parsed_matrix.append(parse_sparse_matrix_entry(line))

    return defs, parsed_matrix

def repl_key(m):
    pattern = m.group(0)
    return pattern + "["

def repl_elem(m):
    pattern = m.group(0)
    return re.sub("[a-z]{4,5}", repl_key, pattern)+"]"

def parse_expression(expr):
    """."""
    patterns = list(set(re.findall("xopt[0-9]{0,9}", expr)))
    patterns = sorted(patterns, key=len)
    for pattern in list(reversed(patterns)):
        expr = re.sub(pattern, repl_elem, expr)

    patterns = list(set(re.findall("param[0-9]{0,9}", expr)))
    patterns = sorted(patterns, key=len)
    for pattern in list(reversed(patterns)):
        expr = re.sub(pattern, repl_elem, expr)

    patterns = list(set(re.findall("lamg[0-9]{0,9}", expr)))
    patterns = sorted(patterns, key=len)
    for pattern in list(reversed(patterns)):
        expr = re.sub(pattern, repl_elem, expr)

    if len(expr.split(",")) > 1:
        parts = expr.split(",")
        for p in range(len(parts)-1):
            def_key_value = parts[p].split('=')
            parts[-1] = parts[-1].replace(def_key_value[0], def_key_value[1])

        expr = parts[-1]
    return expr

def parse_constraint_formulation(line):
    """Expects: In one line a list of equations [(xopt0*xopt1), sq(xopt0)]."""
    lines = line.split(",")
    for l, line_content in enumerate(lines):
        lines[l] = line_content.replace("[", "").replace("]", "")

    defs, lines = find_definitions(lines)
    for l, line_content in enumerate(lines):
        lines[l] = parse_expression(line_content)

    for d in defs:
        defs[d] = parse_expression(defs[d])

    return defs, lines

if __name__ == '__main__':
    test = " @1=3.33333e+13,\n @2=16666.7,\n (16, 16) -> @1\n (17, 17) -> @1\n (18, 18) -> @1\n (19, 19) -> @1\n (20, 20) -> @1\n (21, 21) -> @1\n (22, 22) -> @2\n (23, 23) -> @2\n (24, 24) -> @2\n (25, 25) -> @2\n (26, 26) -> @2\n (27, 27) -> @2\n (28, 28) -> @2\n (29, 29) -> @2\n (30, 30) -> @2\n (31, 31) -> @2\n (32, 32) -> @2\n (33, 33) -> @2\n (62, 62) -> @1\n (63, 63) -> @1\n (64, 64) -> @1\n (65, 65) -> @1\n (66, 66) -> @1\n (67, 67) -> @1\n (68, 68) -> @2\n (69, 69) -> @2\n (70, 70) -> @2\n (71, 71) -> @2\n (72, 72) -> @2\n (73, 73) -> @2\n (74, 74) -> @2\n (75, 75) -> @2\n (76, 76) -> @2\n (77, 77) -> @2\n (78, 78) -> @2\n (79, 79) -> @2\n"
    parsed_matrix = parse_matrix_formulation(test.split("\n"))

    test = "((((param63*(xopt47-param60))+(param64*(xopt48-param61)))+(param65*(xopt49-param62)))+xopt78)"
