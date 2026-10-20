"""Glaucus I topside nodes (Python). Runs on the topside laptop / mini PC.

DESIGN CHOICE: Python is fine up here. Nothing topside is in a control loop -
the robot keeps itself safe (MCU watchdog) and keeps sweeping (Pi executor)
without us. Topside is mission logic, mapping, logging and the app gateway,
where iteration speed matters more than microseconds.
"""
