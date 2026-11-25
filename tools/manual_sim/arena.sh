#!/usr/bin/env bash
# Obstacles for manual flying (Gazebo x = East, y = North; the vehicle starts at 0 0 with the nose East).
T=$(cd "$(dirname "$0")/../scenarios" && pwd)
put() { bash $T/spawn_obstacle.sh "$@" > /dev/null; }
put wall_east   20.15   0   5  0.3 16  10 0      box   # front: tall wall 20 m ahead, 16 m wide
put wall_north   2     16.15 5  16  0.3 10 0      box   # left: wall 16 m away
put pole_1       5     -8    5  0.25 0  10 0      cyl   # right: three thin poles (0.5 m diameter)
put pole_2       9    -11    5  0.25 0  10 0      cyl
put pole_3      13     -7    5  0.25 0  10 0      cyl
put gate_south  18.15  13    5  0.3 4   10 0      box   # north-east: gate with a 6 m gap (between y 15 and 21)
put gate_north  18.15  23    5  0.3 4   10 0      box
put wall_west   -15     0    5  0.3 12  10 0.5236 box   # behind: wall turned 30 degrees
put bar_south    -6   -10    4  8  0.3  0.3 0     box   # south-west: horizontal bar at 4 m height
put pillar_nw    -8     9    5  1   1   10 0      box   # north-west: square pillar 1 x 1 m
