#!/bin/bash

. /home/nightwolf/tools/esp-idf/export.sh
OTA_TARGET=BLE2MQTT-36C8 idf.py $@

