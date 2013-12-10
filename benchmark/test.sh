#!/bin/bash
for EDF in 0.0005 0.001 0.002 0.005
do
	for MVR in 0.5 0.6 0.7 0.8 0.9
	do
		for SMH in {20..500..20}
		do
			directory="$EDF-$MVR-$SMH"
			mkdir "evaluationTest"
			mkdir "evaluationTest/$directory"
			mkdir "evaluationTest/$directory/matches"
			./boxes examples/images/box2/IMG* -a "SURF" -E "EPIPOLAR_DISTANCE_FACTOR=$EDF" -E "MATCH_VALID_RATIO=$MVR" -E "SURF_MIN_HESSIAN=$SMH" -p "evaluationTest/$directory/pointcloud.pcd" -m "evaluationTest/$directory/matches/match.jpg"
		done
	done
done
