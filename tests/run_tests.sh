#!/bin/sh
# Host tests for air530.c, with the system cc and fake Pico SDK headers.
# Usage: sh tests/run_tests.sh
set -e
here=$(cd "$(dirname "$0")" && pwd)
out=$(mktemp -d)
cc -std=gnu11 -Wall -Wextra -Werror -I"$here/stubs" -I"$here" -I"$here/.." \
    -o "$out/test_air530" "$here/test_air530.c" "$here/fake_sdk.c" "$here/../air530.c" -lm
"$out/test_air530"
rm -rf "$out"
