#!/bin/sh
for file in `ls -1 | grep tif | sed -e 's/\..*$//'` 
do
    convert "${file}.tif" -transparent white "${file}.png"
    echo "writing ${file}.png"
done

for file in `ls -1 | grep tga | sed -e 's/\..*$//'` 
do
    convert "${file}.tga" -transparent white "${file}.png"
    echo "writing ${file}.png"
done
