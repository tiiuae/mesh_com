#!/bin/sh

(
    cd "$(dirname "$0")"

    sh ./cbma_check_running.sh lower
)
