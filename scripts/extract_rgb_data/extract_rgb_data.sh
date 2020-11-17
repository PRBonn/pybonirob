#!/bin/bash
# Copyright  2020, Miriam Schnell, University of Stutgart
# email: st148346@stud.uni-stuttgart.de
# check if path_to_destination_folder was given while calling the function
if [ "$1" != "" ]; then
	dataset_dir=`cd "$1"; pwd`;
else
    echo "Usage: ./extract_rgb_data.sh path_to_destination_folder"
    exit 1
fi
metadata_dir=`pwd`/metadata
list_days_file=$metadata_dir/list_days.txt
server="http://www.ipb.uni-bonn.de/datasets_IJRR2017/raw_data"
cd $dataset_dir
# loop through all days
while read -r day || [ -n "$day" ]
do
	day_chunks_list_file=$metadata_dir/$day.txt
	# loop through all zip files of the day
	while read -r chunk || [ -n "$chunk" ]
	do
		echo "Downloading $chunk from server."
		chunk_link=$server/$day/$chunk.zip
		echo chunk_link
		wget $chunk_link
		unzip $chunk.zip
		# copy all rgb data into $dataset_dir
		for file in $chunk/camera/jai/rgb; do
			cp -r $file `pwd`
		done
		# delete the rest
		rm -r $chunk.zip
		rm -r $chunk
	done < "$day_chunks_list_file"
done < "$list_days_file"
