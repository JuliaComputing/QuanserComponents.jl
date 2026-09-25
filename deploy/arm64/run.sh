#!/usr/bin/env bash
# Run a command inside the arm64 root file system prepared by `setup.sh`:
# `deploy/arm64/run.sh [--bind DIR]... CMD ARGS...`.
#
# The calling user becomes uid 0 in a new user namespace; no privilege is required. Every
# `--bind DIR` is mounted at the same absolute path inside, so paths written by the host (an
# application directory, a bundle) mean the same thing on both sides. The host's DNS
# configuration is used for network access.
#
# qemu emulates a Cortex-A72, the CPU of the Raspberry Pi 4, unless QEMU_CPU says otherwise.
# Its default model (`max`, qemu 8.2) makes Julia 1.13 fail while writing any package image
# (`UndefRefError` in `enqueue_specializations!`), and it would let code tuned for features
# the Pi lacks run here without error.
set -euo pipefail

ROOTFS=${QUBE_ARM64_ROOTFS:-$HOME/.cache/QuanserComponents/arm64-rootfs}
[ -x "$ROOTFS/bin/sh" ] || { echo "no root file system at $ROOTFS; run deploy/arm64/setup.sh" >&2; exit 1; }

binds=()
while [ "${1:-}" = --bind ]; do
    dir=$(realpath "$2")
    binds+=(--bind "$dir" "$dir")
    shift 2
done

exec bwrap --unshare-user --uid 0 --gid 0 \
    --bind "$ROOTFS" / \
    --proc /proc --dev /dev --tmpfs /tmp \
    --ro-bind /etc/resolv.conf /etc/resolv.conf \
    "${binds[@]}" \
    --setenv HOME /root \
    --setenv PATH /opt/julia/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin \
    --setenv JULIA_DEPOT_PATH /root/.julia: \
    --setenv QEMU_CPU "${QEMU_CPU:-cortex-a72}" \
    "$@"
