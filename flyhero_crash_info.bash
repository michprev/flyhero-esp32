#!/bin/bash

"$IDF_PATH"/components/espcoredump/espcoredump.py -p /dev/ttyUSB0 info_corefile Flyhero_App/build/Flyhero_App.elf
