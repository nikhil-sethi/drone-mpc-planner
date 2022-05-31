#!/usr/bin/env python3
# -*- coding: utf-8 -*-

def matrix_entry_is_linear(formulation):
    """Finds expression of the following pattern: (67, 67) -> (((param9*(xopt47-param6))+(param10*(xopt48-param7)))+(param11*(xopt49-param8)))+xopt69),
    ."""
    elements = formulation.split(" -> ")
    if len(elements) >= 2:
        if elements[1][:1] == "(":
            return True

    return False


def clean_formulation(lines, begin_line, end_line):
    marked_to_be_deleted = []

    for line_idx in range(begin_line, end_line):
        if lines[line_idx].split(" ")[0] == "sparse:":
           marked_to_be_deleted.append(line_idx)

        elif matrix_entry_is_linear(lines[line_idx]):
           marked_to_be_deleted.append(line_idx)

    for line_idx in list(reversed(marked_to_be_deleted)):
        del lines[line_idx]

    return lines

