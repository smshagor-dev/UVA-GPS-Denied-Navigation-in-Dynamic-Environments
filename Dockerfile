FROM golang:1.22-bookworm AS builder

WORKDIR /src

COPY go.mod ./
COPY cmd ./cmd
COPY internal ./internal

RUN --mount=type=cache,target=/go/pkg/mod \
    --mount=type=cache,target=/root/.cache/go-build \
    CGO_ENABLED=0 GOOS=linux GOARCH=amd64 go build -trimpath -ldflags="-s -w" -o /out/control-plane ./cmd/control-plane

FROM debian:bookworm-slim

RUN apt-get update \
    && apt-get install -y --no-install-recommends ca-certificates curl tzdata \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /app

COPY --from=builder /out/control-plane /usr/local/bin/control-plane

RUN groupadd --system drone \
    && useradd --system --gid drone --create-home --home-dir /home/drone drone \
    && mkdir -p /app/logs/control-plane /app/certs \
    && chown -R drone:drone /app /home/drone

ENV DRONE_SWARM_ADDR=:8080
ENV DRONE_BACKEND_MODE=production
ENV DRONE_BACKEND_SIMULATION_ENABLED=false

EXPOSE 8080

HEALTHCHECK --interval=30s --timeout=5s --start-period=10s --retries=3 \
  CMD curl --fail http://127.0.0.1:8080/api/v1/health || exit 1

USER drone

ENTRYPOINT ["/usr/local/bin/control-plane"]
