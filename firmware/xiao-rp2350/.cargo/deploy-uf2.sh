#!/usr/bin/env bash
set -euo pipefail

if (( $# != 1 )); then
    echo "usage: $0 <firmware ELF>" >&2
    exit 2
fi

if ! command -v picotool >/dev/null 2>&1; then
    echo "error: picotool 2.x is required to package RP2350 UF2 images" >&2
    exit 1
fi

mounts=()
while IFS= read -r -d '' info_file; do
    if grep -q '^Model: Raspberry Pi RP2350$' "$info_file"; then
        mount_dir=${info_file%/INFO_UF2.TXT}
        if [[ -w "$mount_dir" ]]; then
            mounts+=("$mount_dir")
        fi
    fi
done < <(find /run/media /media -maxdepth 4 -type f -name INFO_UF2.TXT -print0 2>/dev/null)

if (( ${#mounts[@]} == 0 )); then
    echo "error: no mounted RP2350 BOOTSEL drive found" >&2
    echo "hold BOOT, tap RESET, release BOOT, and mount the RP2350 drive" >&2
    exit 1
fi

if (( ${#mounts[@]} > 1 )); then
    printf 'error: multiple mounted RP2350 BOOTSEL drives found:\n' >&2
    printf '  %s\n' "${mounts[@]}" >&2
    exit 1
fi

uf2_file=$(mktemp --suffix=.uf2)
trap 'rm -f "$uf2_file"' EXIT

echo "Packaging RP2350 Arm UF2..."
picotool uf2 convert "$1" -t elf "$uf2_file" -t uf2 \
    --family rp2350-arm-s --platform rp2350

echo "Copying firmware to ${mounts[0]}..."
cp "$uf2_file" "${mounts[0]}/firmware.uf2"

# A successful UF2 download normally reboots the board and removes the mount.
sync -f "${mounts[0]}" 2>/dev/null || true
echo "Flash complete; the RP2350 should now be running the firmware."
