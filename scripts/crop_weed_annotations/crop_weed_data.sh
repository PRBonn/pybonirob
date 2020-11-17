#!/bin/bash
# Copyright  2020, Miriam Schnell, University of Stutgart
# email: st148346@stud.uni-stuttgart.de
# check if path_to_destination_folder was given while calling the function
if [ "$1" != "" ]; then
	dataset_dir=`cd "$1"; pwd`;
else
    echo "Usage: ./crop_weed_data.sh path_to_destination_folder"
    exit 1
fi
# set path to metadata
metadata_dir=`pwd`/crop_weed_metadata
server="http://www.ipb.uni-bonn.de/datasets_IJRR2017/annotations/cropweed"
part_chunks_list_file=$metadata_dir/crop_weed_metadata.txt
cd $dataset_dir
# loop through all rar archives
while read -r chunk || [ -n "$chunk" ]
do
	echo "Downloading $chunk from server."
	chunk_link=$server/$chunk.rar
	echo chunk_link
	wget $chunk_link
	unrar x $chunk.rar
	rm -r $chunk.rar
done < "$part_chunks_list_file"

# extract rgb and dlp folders directly into destination folder
list_file=$metadata_dir/list_folders.txt
while read LINE
do
	echo "Copying $LINE to $dataset_dir."
	for file in ijrr_sugarbeets_2016_annotations/$LINE/images/rgb; do
		cp -r $file `pwd`
	done
	for file in ijrr_sugarbeets_2016_annotations/$LINE/annotations/dlp/colorCleaned; do
		cp -r $file `pwd`
	done
	for file in ijrr_sugarbeets_2016_annotations/$LINE/annotations/dlp/iMapCleaned; do
		cp -r $file `pwd`
	done
done < "$list_file"
# delete the rest
rm -r ijrr_sugarbeets_2016_annotations
