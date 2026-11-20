#!/usr/bin/env bash
# Generate a self-signed lab CA + server/client certificates for Mosquitto mTLS.
# LAB USE ONLY: keys are unencrypted and the CA is not trusted by anything else.
set -euo pipefail
DIR="$(cd "$(dirname "$0")/.." && pwd)/certs"
DAYS=825
mkdir -p "$DIR"; cd "$DIR"
if [[ -f ca.crt && "${1:-}" != "--force" ]]; then echo "certs exist in $DIR (use --force to regenerate)"; exit 0; fi
openssl req -x509 -newkey rsa:2048 -nodes -keyout ca.key -out ca.crt -days 3650 \
  -subj "/O=iiot-tank-monitoring lab/CN=iiot-lab-ca" \
  -addext "basicConstraints=critical,CA:TRUE" -addext "keyUsage=critical,keyCertSign,cRLSign" 2>/dev/null
issue() {  # name  CN  extKeyUsage  [SAN]
  local name=$1 cn=$2 eku=$3 san=${4:-}
  openssl req -newkey rsa:2048 -nodes -keyout "$name.key" -out "$name.csr" -subj "/O=iiot-tank-monitoring lab/CN=$cn" 2>/dev/null
  {
    echo "basicConstraints=CA:FALSE"
    echo "keyUsage=critical,digitalSignature,keyEncipherment"
    echo "extendedKeyUsage=$eku"
    [[ -n "$san" ]] && echo "subjectAltName=$san"
  } > "$name.ext"
  openssl x509 -req -in "$name.csr" -CA ca.crt -CAkey ca.key -CAcreateserial -out "$name.crt" -days $DAYS -sha256 -extfile "$name.ext" 2>/dev/null
  rm -f "$name.csr" "$name.ext"
}
issue mosquitto mosquitto serverAuth "DNS:mosquitto,DNS:localhost,IP:127.0.0.1"
issue gateway   gateway   clientAuth
issue ingestor  ingestor  clientAuth
issue bench     bench     clientAuth
chmod 644 ./*.crt ./*.key   # containers run as non-root users; lab only
rm -f ca.srl
echo "generated in $DIR:"; ls -1 "$DIR"
