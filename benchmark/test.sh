#!/bin/bash
#  This file is part of the boxes library.
#
#  boxes is free software; you can redistribute it and/or modify it
#  under the terms of the GNU Lesser General Public License as published by
#  the Free Software Foundation; either version 2.1 of the License, or
#  (at your option) any later version.

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
