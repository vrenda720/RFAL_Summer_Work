# !/bin/bash

# find . *.bag > $paths
# echo $paths

bags=($(find . -maxdepth 1 -name '[0-9]*' -type d | sort -n))

for bag in "${bags[@]}"
do
    # shortbag=${bag:0:-4}
    final+="-i "
    final+="$bag"
    final+=" "
    # rosbags-convert --src $bag --dst $shortbag
done
final+="-o output.yaml"
echo $final
# ros2 bag convert $final