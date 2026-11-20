#!/bin/sh
# Fetch pinned third-party sources for the STM32 build into third_party/.
# Re-running is safe: existing checkouts are verified against the pinned commit.
set -eu

ROOT=$(cd "$(dirname "$0")/.." && pwd)
DEST="$ROOT/third_party"
mkdir -p "$DEST"

# name  repo  tag  commit (the tag's peeled commit, not the tag object)  [sparse paths...]
fetch() {
    name=$1 repo=$2 tag=$3 commit=$4
    shift 4
    dir="$DEST/$name"
    if [ ! -d "$dir/.git" ]; then
        if [ $# -gt 0 ]; then
            git -c advice.detachedHead=false clone --quiet --depth 1 --branch "$tag" --filter=blob:none --sparse "https://github.com/$repo.git" "$dir"
            git -C "$dir" sparse-checkout set "$@"
        else
            git -c advice.detachedHead=false clone --quiet --depth 1 --branch "$tag" "https://github.com/$repo.git" "$dir"
        fi
    fi
    actual=$(git -C "$dir" rev-parse HEAD)
    if [ "$actual" != "$commit" ]; then
        echo "error: $name is at $actual, expected $commit ($tag)" >&2
        exit 1
    fi
    echo "$name $tag ok"
}

fetch FreeRTOS-Kernel FreeRTOS/FreeRTOS-Kernel V11.1.0 dbf70559b27d39c1fdb68dfb9a32140b6a6777a0
fetch CMSIS_5 ARM-software/CMSIS_5 5.9.0 2b7495b8535bdcb306dac29b9ded4cfb679d7e5c CMSIS/Core/Include
fetch cmsis_device_f4 STMicroelectronics/cmsis_device_f4 v2.6.10 5f41fb29d22773896c780052bf61e47fc924d524
