#!/bin/sh
# Virtual serial pair: /tmp/ttyNODE (emulated ESP32 UART) <-> /tmp/ttyBUS (wire side)
( while true; do socat -d PTY,raw,echo=0,link=/tmp/ttyNODE PTY,raw,echo=0,link=/tmp/ttyBUS; sleep 1; done ) &
while [ ! -e /tmp/ttyBUS ]; do sleep 0.1; done
exec python services/vibnode/emulator.py
