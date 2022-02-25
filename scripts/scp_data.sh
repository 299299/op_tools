#!/bin/bash

IP=$1
OUTPUT_FOLDER=$2

#scp -r root@$IP:/data/media/0/realdata $2
scp -r root@$IP:/data/media/0/dashcam $OUTPUT_FOLDER