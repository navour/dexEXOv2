#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")"
mkdir -p bin
g++ -std=c++17 -O2 -Wall -Wextra -pthread \
    mhandpro_diagnostic.cpp -ldl -o bin/mhandpro_diagnostic
echo "Built: $(pwd)/bin/mhandpro_diagnostic"
