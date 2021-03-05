#!/usr/bin/env python3
import time, argparse,datetime
from jsons_to_db import jsons_to_db
from jsons_to_LG import jsons_to_LG

parser = argparse.ArgumentParser(description='Script that adds the json files to an sql database.')
parser.add_argument('-i', '--input_folder', help="Path to the folder with json files", default='~/jsons/')
parser.add_argument('-p','--period', help="Repeat this script every period", default=3600)
parser.add_argument('-o','--output_db_path', help="Path to sql database", default='~/patsc/db/pats.db')
args = parser.parse_args()

while True:
    jsons_to_db(args.input_folder,args.output_db_path)
    jsons_to_LG(args.input_folder)

    if args.period:
        print(str(datetime.datetime.now()) + ". Periodic update after: " + str(args.period))
        time.sleep(int(args.period))
    else:
        break
