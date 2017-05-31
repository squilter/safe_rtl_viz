# safe_rtl_viz

This repository is the first steps towards my 2017 GSoC project, "Safe Return-to-Launch for ArduPilot". The python script, `visualizer.py`, takes a standard .log file and animates the copter flying, and the result of the Safe-RTL algorithm. To test it out, run:

`./visualizer.py ./logs/robert_lefebvre_octo_PM.log`

How the current version works:

- create an array that represents the path that we will fly to get back home. (TODO prevent length from increasing beyond 20!)
- the first item in this array should be the home position
- Every time a new position comes in, check if our current position is more than two meters from the last item in the array
    - if it is, append the current position to the array
- If the array is almost full, run the following cleanup algorithm:
    - check if the path overlaps (or almost overlaps) with itself (pruning step)
        - if so, slice out any points between those.
    - If a point B lies between points A and C, remove B (simplification step)
        - uses the Ramer–Douglas–Peucker algorithm
- When RTL is triggered, run the cleanup algorithm once more, and then fly the path back home.
