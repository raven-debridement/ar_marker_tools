#!/usr/bin/env bash

BEGIN=0
END=15
SCALEBEGIN=1
SCALEEND=5
while getopts ":b:e:s:S:h" opt; do
    case $opt in
	b)
	    BEGIN=$OPTARG
	    ;;
	e)
	    END=$OPTARG
	    ;;
	s)
	    SCALEBEGIN=$OPTARG
	    ;;
	S)
	    SCALEEND=$OPTARG
	    ;;
	h)
	    echo "Usage: multiscale.bash [-b MARKERBEGIN] [-e MARKEREND] [-s SCALEBEGIN] [-S SCALEEND]"
	    ;;
    esac
done
echo "Printing markers from $BEGIN to $END at scales from $SCALEBEGIN to $SCALEEND"

for ((i=SCALEBEGIN; i<=SCALEEND; i++))
do
    echo "Generating image $i"
    rosrun marker_print print_markers.py -q -b $BEGIN -e $END -s $i -o markers$i.png
done