---
description: Change the sweep algorithm the safe way (Python sim first, then C++ port, then parity)
argument-hint: <what to change>
---
Change requested: $ARGUMENTS

Follow the workflow in CLAUDE.md rule 2 and docs/decisions/0006:

1. Implement the change in `sim/sweep_sim.py`. Keep names and order of
   operations easy to mirror in C++.
2. Run `python3 sim/sweep_sim.py` and at least seeds 1-10 with `--quiet`.
   Report coverage and x-error before vs. after. Look at the plot.
3. If it's worse, say so and stop. Don't port a regression.
4. Port to `ros2_ws/src/glaucus_core` line for line. Add or adjust a unit test
   in `test/test_sweep_controller.cpp` for the new behavior.
5. If you added parameters: add them to Python `SweepParams`, C++
   `SweepParams`, `config/sweep_params.yaml`, and `sweep_executor_node.cpp`'s
   `load_params()`.
6. Regenerate the parity trace and run `./tools/check_all.sh`.
7. Update `docs/sweep_algorithm.md` so the plain-language explanation still
   matches the code.
