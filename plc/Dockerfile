# OpenPLC v3 runtime, built from source for arm64 (no official arm64 image),
# plus a headless provisioner that uploads/compiles tank_control.st, adds the
# simulated plant as a Modbus TCP slave device, and starts the PLC.
FROM debian:trixie-20251020

ARG OPENPLC_REPO=https://github.com/thiagoralves/OpenPLC_v3.git
ARG OPENPLC_COMMIT=b5d41356dab4aeadca0dd7ca64ba542f870b595d

RUN apt-get update && apt-get install -y --no-install-recommends git ca-certificates \
    && git clone ${OPENPLC_REPO} /workdir && cd /workdir && git checkout ${OPENPLC_COMMIT} \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /workdir
RUN mkdir /docker_persistent
# identical to upstream Dockerfile at that commit
RUN ./install.sh docker \
    && touch /docker_persistent/mbconfig.cfg \
    && touch /docker_persistent/persistent.file \
    && mkdir /docker_persistent/st_files \
    && cp /workdir/webserver/openplc.db /docker_persistent/openplc.db \
    && mv /workdir/webserver/openplc.db /workdir/webserver/openplc_default.db \
    && cp /workdir/webserver/dnp3.cfg /docker_persistent/dnp3.cfg \
    && mv /workdir/webserver/dnp3.cfg /workdir/webserver/dnp3_default.cfg \
    && cp -r /workdir/webserver/st_files/ /docker_persistent/st_files/ \
    && mv /workdir/webserver/st_files /workdir/webserver/st_files_default \
    && cp /workdir/webserver/active_program /docker_persistent/active_program \
    && mv /workdir/webserver/active_program /workdir/webserver/active_program_default \
    && ln -s /docker_persistent/mbconfig.cfg /workdir/webserver/mbconfig.cfg \
    && ln -s /docker_persistent/persistent.file /workdir/webserver/persistent.file \
    && ln -s /docker_persistent/openplc.db /workdir/webserver/openplc.db \
    && ln -s /docker_persistent/dnp3.cfg /workdir/webserver/dnp3.cfg \
    && ln -s /docker_persistent/st_files /workdir/webserver/st_files \
    && ln -s /docker_persistent/active_program /workdir/webserver/active_program \
    && rm -rf /var/lib/apt/lists/*

COPY tank_control.st provision.py entrypoint.sh /opt/iiot/
RUN chmod +x /opt/iiot/entrypoint.sh
EXPOSE 8080 502
HEALTHCHECK --interval=5s --timeout=3s --start-period=20s --retries=60 \
    CMD test -f /tmp/plc_provisioned
ENTRYPOINT ["/opt/iiot/entrypoint.sh"]
