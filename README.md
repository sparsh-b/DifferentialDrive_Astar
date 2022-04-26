# About
- Path planning with A\* for a finite-radius mobile robot with non-holonomic constraints.
- Due to the consideration of non-holonomic constraints while planing the path itself, the generated path will be kinematically executable with no sharp points.
- Rpms of 5 & 10 for the left & right wheels were used & the map is scaled down to its 40th part to decerease run-time.
- This work is jointly done by Sparsh Bhogavilli & Aneesh Dandime as part of Project3 of the course ENPM661 at UMD.

# Instructions to run
- `cd scripts`
- python3 diff_drive_Astar.py --start <space-separated x y theta> --end <space-separated x y> --rpm <space-separated rpm1 & rpm2> --clearance <clearance in mm>
- In the above command, x & y are mm, theta is in degrees
- x & y valid range is [0, 9999].
- Example: `python3 diff_drive_Astar.py --start 1000 1000 270 --goal 9000 9000 --rpm 5 10 --clearance 5`
