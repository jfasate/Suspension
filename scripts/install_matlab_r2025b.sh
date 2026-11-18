#!/usr/bin/env bash
set -euo pipefail
# Side-by-side user installation; activation requires an eligible MathWorks license.
installer_dir="${XDG_DATA_HOME:-$HOME/.local/share}/mathworks-install"
destination="${1:-$HOME/MATLAB/R2025b}"
mkdir -p "$installer_dir"
curl -fL --retry 2 https://www.mathworks.com/mpm/glnxa64/mpm -o "$installer_dir/mpm"
chmod u+x "$installer_dir/mpm"
"$installer_dir/mpm" install --release=R2025b --destination="$destination" \
    --products=MATLAB Simulink Symbolic_Math_Toolbox Fuzzy_Logic_Toolbox
