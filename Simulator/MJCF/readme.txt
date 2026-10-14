In order to generate the scene with stairs it is needed to use the command:

python3 /home/yoggi/mors_mpc/Simulator/MJCF/generate_stairs.py \
  --step-height 0.05 \
  --step-length 0.3 \
  --step-count 5 \
  --top-platform-size 1.0


In order to generate the scene with free-falling boards it is needed to use the command:

python3 /home/yoggi/mors_mpc/Simulator/MJCF/generate_boards.py \
  --board-count 12 \
  --mean-length 0.7 \
  --mean-thickness 0.05 \
  --mean-width 0.25 \
  --x-span 10.0 \
  --mixed-materials \
  --seed 7

python3 /home/yoggi/mors_mpc/Simulator/MJCF/generate_boxes.py \
  --first-box-height 0.01 \
  --last-box-height 0.12 \
  --box-count 12 \
  --center-spacing 0.9

python3 /home/yoggi/mors_mpc/Simulator/MJCF/generate_stumps.py \
  --platform-width 2.0 \
  --step-count 5 \
  --step-height 0.05 \
  --step-depth 0.3 \
  --top-platform-length 0.8 \
  --stump-zone-length 1.8 \
  --max-stump-gap 0.25 \
  --min-stump-gap 0.05 \
  --stump-height-variation 0.08 \
  --min-stump-diameter 0.08 \
  --max-stump-diameter 0.2 \
  --seed 7

python3 /home/yoggi/mors_mpc/Simulator/MJCF/generate_platform.py \
  --height 0.1 \
  --length 1.0 \
  --width 2.0 \
  --center-x 1.5

python3 /home/yoggi/mors_mpc/Simulator/MJCF/generate_ramp.py \
  --slope-angle 35 \
  --height 0.5 \
  --flat-length 2

python3 /home/yoggi/mors_mpc/Simulator/MJCF/generate_building.py \
  --wall-height 2.0 \
  --mezzanine-height 0.6 \
  --ramp-angle 12 \
  --stair-step-height 0.05 \
  --stair-step-depth 0.25 \
  --door-width 1.0 \
  --box-first-height 0.02 \
  --box-last-height 0.08 \
  --board-count 4 \
  --board-thickness 0.04 \
  --seed 7 \
  --ceiling

# building_obstacle.xml: the same scene plus a box missing from the navigation map
# (obstacle avoidance test); the other arguments must match building.xml
python3 /home/yoggi/mors_mpc/Simulator/MJCF/generate_building.py \
  --box-first-height 0.01 \
  --box-last-height 0.01 \
  --board-count 1 \
  --board-thickness 0.01 \
  --obstacle \
  --output /home/yoggi/mors_mpc/Simulator/MJCF/building_obstacle.xml
