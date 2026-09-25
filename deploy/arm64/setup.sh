#!/usr/bin/env bash
# Prepare a rootless arm64 Debian bookworm root file system in which a JuliaC application is
# built for a 64-bit Raspberry Pi: `deploy/arm64/setup.sh [ROOTFS]`.
#
# JuliaC cannot cross-compile. The package image of the application is produced by the Julia
# that runs the build, so the build must run aarch64 Julia. On an x86_64 host this relies on
# qemu-user through binfmt_misc (the `qemu-user-static` or `qemu-user-binfmt` package, whose
# registration carries the F flag, so the interpreter is found from inside the namespace).
#
# Nothing here requires root or a container daemon. The root file system is the arm64 layer of
# Docker Hub's `debian:bookworm`, fetched over the registry's HTTP API, and it is entered with
# `run.sh`, which maps the calling user to uid 0 in a user namespace. That namespace maps a
# single uid, so apt is told not to drop privileges to `_apt`.
#
# Installed into it:
#   gcc, make           the C compiler JuliaC links with, and the build of csrc/
#   libhil1-dev         the arm64 HIL SDK, from Quanser's apt repository (the one the Pi uses)
#   Julia $JULIA_VERSION  under /opt/julia, and JuliaC in the environment /opt/juliac
#
# Re-running is safe: every step is skipped when its result already exists.
set -euo pipefail

ROOTFS=${1:-${QUBE_ARM64_ROOTFS:-$HOME/.cache/QuanserComponents/arm64-rootfs}}
JULIA_VERSION=${JULIA_VERSION:-1.13.0}
JULIAC_VERSION=${JULIAC_VERSION:-0.3.10}
HERE=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
RUN="$HERE/run.sh"
export QUBE_ARM64_ROOTFS=$ROOTFS

if [ ! -e /proc/sys/fs/binfmt_misc/qemu-aarch64 ] && [ "$(uname -m)" != aarch64 ]; then
    echo "no qemu-aarch64 binfmt_misc registration; install qemu-user-static" >&2
    exit 1
fi

# --- root file system -----------------------------------------------------------------------
if [ ! -x "$ROOTFS/bin/bash" ]; then
    echo "fetching debian:bookworm (linux/arm64) into $ROOTFS"
    mkdir -p "$ROOTFS"
    registry=https://registry-1.docker.io/v2/library/debian
    token=$(curl -fsS "https://auth.docker.io/token?service=registry.docker.io&scope=repository:library/debian:pull" |
        python3 -c 'import json, sys; print(json.load(sys.stdin)["token"])')
    auth="Authorization: Bearer $token"
    accept='Accept: application/vnd.oci.image.index.v1+json, application/vnd.docker.distribution.manifest.list.v2+json, application/vnd.oci.image.manifest.v1+json, application/vnd.docker.distribution.manifest.v2+json'
    manifest=$(curl -fsS -H "$auth" -H "$accept" "$registry/manifests/bookworm" |
        python3 -c 'import json, sys
d = json.load(sys.stdin)
print(next(m["digest"] for m in d["manifests"] if m["platform"].get("architecture") == "arm64"))')
    curl -fsS -H "$auth" -H "$accept" "$registry/manifests/$manifest" |
        python3 -c 'import json, sys; [print(l["digest"]) for l in json.load(sys.stdin)["layers"]]' |
        while read -r layer; do
            curl -fsSL -H "$auth" "$registry/blobs/$layer" | tar -xz -C "$ROOTFS" --no-same-owner --exclude='dev/*'
        done
fi

# --- packages -------------------------------------------------------------------------------
if [ ! -x "$ROOTFS/usr/bin/gcc" ] || [ ! -e "$ROOTFS/usr/include/quanser/hil.h" ]; then
    echo "installing gcc, make and the HIL SDK"
    mkdir -p "$ROOTFS/etc/apt/apt.conf.d" "$ROOTFS/etc/apt/sources.list.d" "$ROOTFS/usr/share/keyrings"
    echo 'APT::Sandbox::User "root";' > "$ROOTFS/etc/apt/apt.conf.d/99rootless"
    # Debian's own sources first: Quanser's repository is served over https, which needs
    # ca-certificates installed before apt can verify it.
    "$RUN" sh -c 'apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
        gcc make libc6-dev ca-certificates curl file binutils'
    curl -fsS https://repo.quanser.com/keys/Quanser.pub | gpg --dearmor > "$ROOTFS/usr/share/keyrings/Quanser.gpg"
    curl -fsS https://repo.quanser.com/debian/release/config/quanser_raspbian64.sources \
        > "$ROOTFS/etc/apt/sources.list.d/quanser_raspbian64.sources"
    curl -fsS https://repo.quanser.com/debian/release/config/99-quanser-raspbian64 \
        > "$ROOTFS/etc/apt/preferences.d/99-quanser-raspbian64"
    "$RUN" apt-get update
    # The development packages of the HIL API, which is what `csrc/qube_hw.c` links against. The
    # `quanser-sdk` metapackage would add the Python bindings and camera drivers. A postinst script updates the udev
    # hardware database, which a build root file system has no use for, so `systemd-hwdb` is
    # stubbed out.
    printf '#!/bin/sh\nexit 0\n' > "$ROOTFS/usr/local/bin/systemd-hwdb"
    chmod +x "$ROOTFS/usr/local/bin/systemd-hwdb"
    "$RUN" sh -c 'DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
        libhil1-dev libquanser-runtime1-dev libquanser-common1-dev'
fi

# --- Julia and JuliaC -----------------------------------------------------------------------
if [ ! -x "$ROOTFS/opt/julia/bin/julia" ]; then
    echo "installing Julia $JULIA_VERSION (aarch64)"
    minor=${JULIA_VERSION%.*}
    mkdir -p "$ROOTFS/opt/julia"
    curl -fsSL "https://julialang-s3.julialang.org/bin/linux/aarch64/$minor/julia-$JULIA_VERSION-linux-aarch64.tar.gz" |
        tar -xz -C "$ROOTFS/opt/julia" --strip-components=1
    ln -sf /opt/julia/bin/julia "$ROOTFS/usr/local/bin/julia"
fi
if [ ! -e "$ROOTFS/opt/juliac/Manifest.toml" ]; then
    echo "installing JuliaC $JULIAC_VERSION (slow under emulation)"
    mkdir -p "$ROOTFS/opt/juliac"
    "$RUN" julia --startup-file=no --project=/opt/juliac -e \
        "using Pkg; Pkg.add(name=\"JuliaC\", version=\"$JULIAC_VERSION\"); Pkg.precompile()"
fi
echo "arm64 root file system ready: $ROOTFS"
