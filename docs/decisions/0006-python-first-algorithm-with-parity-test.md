# 0006: Prototype the algorithm in Python, port to C++, prove it with a parity test

**Status:** accepted

## Decision
`sim/sweep_sim.py` is the algorithm's design surface. Changes land there
first, get checked against the plot and several seeds, then get ported 1:1 to
`glaucus_core`. `test_parity` replays a Python trace through the C++ controller
and fails on any differing command.

## Why
- Iterating on a state machine with a plot in seconds beats iterating with a
  tank in hours.
- The robot runs C++ (as wanted for controls), but nobody has to wonder
  whether the C++ matches what was tested.

## Rules
- Same names, same order of operations, same defaults in three places:
  Python `SweepParams`, C++ `SweepParams`, `config/sweep_params.yaml`.
  `tools/check_params.py` enforces it.
- After changing the controller in either language, regenerate the trace and
  run `tools/check_all.sh`.
