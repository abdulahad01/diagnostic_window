#!/usr/bin/env python
import subprocess

sensors = {'ouster': '192.0.2.0'}
def ping():
    for sensor in sensors:
        try:
            subprocess.check_output(["ping", "-c", "1", sensors[sensor]])
            return True
        except subprocess.CalledProcessError:
            return False

print(ping())