#!/usr/bin/env bash

DIR=`rospack find marker_gazebo`/Media/materials/textures
mkdir -p $DIR
mkdir -p `rospack find marker_gazebo`/Media/materials/scripts
SCRIPT=`rospack find marker_gazebo`/Media/materials/scripts/Gazebo.material

#Initialize the file
:>$SCRIPT
for ((i=1;i<=$1;++i)); do
    rosrun marker_print print_markers.py -b $i -e $i -q -o $DIR/bch$i.png
    echo "material BCH/Marker$i" >> $SCRIPT
    echo "{" >> $SCRIPT
    echo "  technique" >> $SCRIPT
    echo "  {" >> $SCRIPT
    echo "    pass" >> $SCRIPT
    echo "    {" >> $SCRIPT
    echo "      texture_unit" >> $SCRIPT
    echo "      {" >> $SCRIPT
    echo "        texture bch$i.png" >> $SCRIPT
    echo "      }" >> $SCRIPT
    echo "    }" >> $SCRIPT
    echo "  }" >> $SCRIPT
    echo "} " >> $SCRIPT
done
