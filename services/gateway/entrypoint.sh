#!/bin/sh
# Gateway's serial port: socat PTY attached to the RS-485 "wire" (TCP, like a
# serial device server in raw mode). Reconnects if the wire drops.
( while true; do socat PTY,raw,echo=0,link=/tmp/ttyRS485 TCP:${VIB_WIRE_HOST:-vibnode}:${VIB_WIRE_PORT:-7000},nodelay; sleep 1; done ) &
exec python services/gateway/main.py
