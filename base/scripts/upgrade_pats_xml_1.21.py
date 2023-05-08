import os
import glob
import xml.etree.ElementTree as ET
import xmltodict
from collections import OrderedDict


def modify_xml(file_path):
    with open(file_path, 'r') as f:
        xml_dict = xmltodict.parse(f.read())

    # Change the version to 1.21
    xml_dict['SerializableClass'].update({'@Version': '1.21'})

    # Add new members
    new_members = [
        {'Name': 'charging', 'value': 'false'},
        {'Name': 'tag', 'value': 'v2.3.0'},
        {'Name': 'trapeye', 'value': 'false'}
    ]

    if 'Member' not in xml_dict['SerializableClass']:
        xml_dict['SerializableClass']['Member'] = []

    for member in new_members:
        new_member = OrderedDict([('@Name', member['Name']), ('#text', member['value'])])
        xml_dict['SerializableClass']['Member'].append(new_member)

    xml_string = xmltodict.unparse(xml_dict, pretty=4)
    xml_string = '\n'.join(xml_string.split('\n')[1:])
    xml_string = xml_string.replace('\t', ' ' * 4)

    with open(file_path, 'w') as f:
        f.write(xml_string)


def main():
    xml_folder = os.path.expanduser('~/Downloads/xml')
    xml_files = glob.glob(os.path.join(xml_folder, '*.xml'))

    for xml_file in xml_files:
        modify_xml(xml_file)

if __name__ == '__main__':
    main()
