#!/bin/bash

echo "****** Running ours method"
python simulation.py --method ours --num_drones 1
python simulation.py --method ours --num_drones 2
python simulation.py --method ours --num_drones 3
python simulation.py --method ours --num_drones 4

echo "****** Running random method"
python simulation.py --method random --num_drones 1
python simulation.py --method random --num_drones 2
python simulation.py --method random --num_drones 3
python simulation.py --method random --num_drones 4

echo "****** Running distance method"
python simulation.py --method distance --num_drones 1
python simulation.py --method distance --num_drones 2
python simulation.py --method distance --num_drones 3
python simulation.py --method distance --num_drones 4

echo "****** Running price method"
python simulation.py --method price --num_drones 1
python simulation.py --method price --num_drones 2
python simulation.py --method price --num_drones 3
python simulation.py --method price --num_drones 4