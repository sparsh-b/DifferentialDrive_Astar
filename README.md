# About
- This work is jointly done Sparsh Bhogavilli & Aneesh Dandime as part of Project3 of the course ENPM661 at UMD.

# Instructions to run
- python3 diff_drive_Astar.py --start <space-separated x y theta> --end <space-separated x y> --rpm <space-separated rpm1 & rpm2> --clearance <clearance in mm>
- In the above command, x & y are mm, theta is in degrees
- x & y valid range is [0, 9999].
- Example: `python3 diff_drive_Astar.py --start 1000 1000 30 --goal 9000 9000 --rpm 100 200 --clearance 5`
