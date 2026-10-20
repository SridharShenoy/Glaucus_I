---
description: Run every check that works without ROS or hardware, and fix what fails
---
Run `./tools/check_all.sh`. If anything fails, find the root cause and fix it
(don't weaken the check). If the parity trace is stale because the algorithm
intentionally changed, confirm the Python and C++ controllers still match line
for line, then regenerate the trace as the script instructs. Report what failed
and what you changed.
