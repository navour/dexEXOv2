#!/usr/bin/env bash
set -euo pipefail

project_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [[ -x "${project_dir}/.venv/bin/python" ]]; then
    python_bin="${project_dir}/.venv/bin/python"
elif [[ -x "${project_dir}/../PC端/.venv/bin/python" ]]; then
    python_bin="${project_dir}/../PC端/.venv/bin/python"
else
    python_bin="python3"
fi

exec "${python_bin}" "${project_dir}/mhandpro_studio.py" "$@"
