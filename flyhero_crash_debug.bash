#!/bin/bash

"$IDF_PATH"/components/espcoredump/espcoredump.py -p /dev/ttyUSB0 dbg_corefile Flyhero_App/build/Flyhero_App.elf
