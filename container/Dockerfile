FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive
ENV AP_MESH_BUILD_JOBS=2
ENV AP_MESH_CLEAN_BUILD=1

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
       build-essential \
       ca-certificates \
       cmake \
       python3 \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /workspace
COPY . .
RUN chmod +x reproduce_eistute_linux.sh scripts/*.sh \
    && bash scripts/build_linux.sh

CMD ["bash", "reproduce_eistute_linux.sh"]
