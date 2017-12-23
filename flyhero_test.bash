#!/bin/bash

if [ $# -eq 0 ]; then
	cd components
	set "$(echo *)"
	cd ..
fi

make flash monitor -C ${IDF_PATH}/tools/unit-test-app EXTRA_COMPONENT_DIRS="$PWD"/components/ TEST_COMPONENTS="$*"
