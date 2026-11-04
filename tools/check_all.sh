#!/usr/bin/env bash
# Everything that can be checked without ROS or hardware. Run before every push.
#   ./tools/check_all.sh
# Needs: python3 (numpy, matplotlib; fastapi for the gateway check), cmake, g++.
set -euo pipefail
cd "$(dirname "$0")/.."
BUILD="${BUILD_DIR:-build}"
TMP="$(mktemp -d)"
trap 'rm -rf "$TMP"' EXIT
step() { printf '\n== %s\n' "$1"; }

step "parameter defaults in sync (Python / C++ / YAML)"
python3 tools/check_params.py

step "simulator runs and covers the hull"
python3 sim/sweep_sim.py --quiet --out "$TMP/sweep.png" --dump-trace "$TMP/trace.csv" | tee "$TMP/sim.txt"
python3 - "$TMP/sim.txt" <<'EOF'
import re, sys
cov = float(re.search(r"coverage=([\d.]+)%", open(sys.argv[1]).read()).group(1))
assert cov >= 95.0, f"coverage {cov}% < 95%"
EOF

step "3D simulator: geometry, plane == 2D sim, curved-hull sweep"
python3 sim/test_sim3d.py

step "committed parity trace is up to date"
if ! cmp -s "$TMP/trace.csv" ros2_ws/src/glaucus_core/test/data/sim_trace_seed2.csv; then
  echo "Trace is stale. Regenerate with:"
  echo "  python3 sim/sweep_sim.py --quiet --dump-trace ros2_ws/src/glaucus_core/test/data/sim_trace_seed2.csv"
  exit 1
fi
echo "ok"

step "glaucus_core: build + unit tests + Python/C++ parity"
cmake -S ros2_ws/src/glaucus_core -B "$BUILD/core" -DCMAKE_BUILD_TYPE=Release >/dev/null
cmake --build "$BUILD/core" -j >/dev/null
ctest --test-dir "$BUILD/core" --output-on-failure

step "Pi<->MCU protocol"
g++ -std=c++17 -O1 -Wall -Wextra shared/protocol/test_protocol.cpp -o "$TMP/tp" && "$TMP/tp"

step "firmware heading estimator/controller (host build)"
g++ -std=c++17 -O1 -Wall -Wextra -I firmware/src firmware/test/test_heading_host.cpp -o "$TMP/th" && "$TMP/th"

step "Python syntax (ROS nodes, launch files, gateway)"
python3 -m py_compile $(git ls-files '*.py' 2>/dev/null || find . -name '*.py' -not -path './build/*')
echo "ok"

step "web gateway imports (mock backend)"
if python3 -c "import fastapi" 2>/dev/null; then
  (cd webapp/gateway && python3 -c "from server import create_app; from backends import MockBackend; create_app(MockBackend()); print('ok')")
else
  echo "skipped (pip install -r webapp/requirements.txt)"
fi

printf '\nall checks passed\n'
