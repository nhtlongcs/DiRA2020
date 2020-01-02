#!/bin/bash

declare -a fileid=(
    "17zgFE_BRWVVqoPRbAjZscu2ZYdNT0mBf"
    "1lEPCqdJG6YLr0kJVViCfON1Bct6AxaIu"
    "1FiOJmCB4i1cE012ga4t7wHIezApht4HC"
    "1qZcSi9mh1qR2RcsIBzfpxNSOKfwOxYsR"
    )

declare -a filename=(
    "weight_dira.h5"
    "model_dira_v2.h5"
    "model_dira_v3.h5"
    "model_dira_v4.h5"
    )

# get length of an array
fileid_len=${#fileid[@]}
filename_len=${#filename[@]}

if [ ! $fileid_len -eq $filename_len ]; then
    echo "Length of fileid and filename must match"
    exit -1
fi

# use for loop to read all values and indexes
for (( i=0; i<${fileid_len}; i++ ));
do
    echo "Downloading " ${filename[$i]}
    wget --load-cookies /tmp/cookies.txt "https://docs.google.com/uc?export=download&confirm=$(wget --quiet --save-cookies /tmp/cookies.txt --keep-session-cookies --no-check-certificate 'https://docs.google.com/uc?export=download&id=${fileid[$i]}' -O- | sed -rn 's/.*confirm=([0-9A-Za-z_]+).*/\1\n/p')&id=${fileid[$i]}" -O ${filename[$i]} && rm -rf /tmp/cookies.txt
    echo ${filename[$i]} " downloaded"
done