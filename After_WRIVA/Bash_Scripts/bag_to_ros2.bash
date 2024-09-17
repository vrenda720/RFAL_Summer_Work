# !/bin/bash

# find . *.bag > $paths
# echo $paths

bags=(*.bag)

for bag in "${bags[@]}"
do
    shortbag=${bag:0:-4}
    # echo $shortbag
    rosbags-convert --src $bag --dst $shortbag
done