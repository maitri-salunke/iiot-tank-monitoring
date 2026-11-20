#!/bin/bash
# Start the stock OpenPLC webserver, then provision it over its own web API.
rm -f /tmp/plc_provisioned
/workdir/start_openplc.sh &
OPENPLC_PID=$!
/workdir/.venv/bin/python3 /opt/iiot/provision.py || { echo "provisioning FAILED"; kill $OPENPLC_PID; exit 1; }
touch /tmp/plc_provisioned
wait $OPENPLC_PID
