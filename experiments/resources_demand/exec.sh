#!/bin/bash

echo "****** Running ours method"
python simulation.py --method ours --resource 100
python simulation.py --method ours --resource 200
python simulation.py --method ours --resource 300
python simulation.py --method ours --resource 400

echo "****** Running random method"
python simulation.py --method random --resource 100
python simulation.py --method random --resource 200
python simulation.py --method random --resource 300
python simulation.py --method random --resource 400

echo "****** Running distance method"
python simulation.py --method distance --resource 100
python simulation.py --method distance --resource 200
python simulation.py --method distance --resource 300
python simulation.py --method distance --resource 400

echo "****** Running price method"
python simulation.py --method price --resource 100
python simulation.py --method price --resource 200
python simulation.py --method price --resource 300
python simulation.py --method price --resource 400