# One small image for plant, vibnode, gateway and ingestor (different commands).
FROM python:3.12-slim-bookworm
RUN apt-get update && apt-get install -y --no-install-recommends socat procps \
    && rm -rf /var/lib/apt/lists/*
WORKDIR /app
COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt
COPY iiot/ iiot/
COPY services/ services/
COPY bench/ bench/
RUN chmod +x services/*/entrypoint.sh 2>/dev/null || true
ENV PYTHONUNBUFFERED=1
