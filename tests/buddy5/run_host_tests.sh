#!/bin/sh
set -eu
cd "$(dirname "$0")/../.."
bin=$(mktemp /tmp/b5-test.XXXXXX)
trap 'rm -f "$bin"' EXIT HUP INT TERM
cc -std=c11 -Wall -Wextra -Werror -Wpedantic -DAPP_BUDDY5=1 -Iapp_program app_program/b5_core.c app_program/b5_codec.c tests/buddy5/test_b5.c -o "$bin"
"$bin"
