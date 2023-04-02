import subprocess
import argparse

parser = argparse.ArgumentParser(description='Script that loads the timezone for a system')
parser.add_argument('--system', help="system that needs it timezone", required=True)
args = parser.parse_args()

cmd = ['/usr/bin/python3', '/home/pats/pats/dash/timezone_loader.py', '--system', args.system]
subprocess.run(cmd)
