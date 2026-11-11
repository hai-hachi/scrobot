#!/usr/bin/env python3
"""Opt-in throttled single-process CPU load. Runs only until launcher exits."""
import signal
import sys
import time

stop=False

def requested(_sig,_frame):
    global stop
    stop=True

if __name__=='__main__':
    duty=float(sys.argv[1])
    if not 0<duty<=.75:raise ValueError('CPU stress duty must be >0 and <=0.75')
    signal.signal(signal.SIGTERM,requested)
    signal.signal(signal.SIGINT,requested)
    period=.1
    while not stop:
        begin=time.monotonic()
        while not stop and time.monotonic()-begin<period*duty:
            pass
        time.sleep(max(.001,period*(1-duty)))
