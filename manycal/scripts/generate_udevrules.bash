#!/bin/bash
# This script creates a udev rules file to assign camera symlinks based on USB
# hardware paths

if [[ $EUID -ne 0 ]]; then
   echo "Root access is required to create the udev rules file. Please rerun with sudo." 
   exit 1
fi

output_file="/etc/udev/rules.d/99-argus-cameras.rules"

# Check existence of output first
if [ -e "$output_file" ]; then
	echo "$output_file already exists."
	exit 1
fi

# For each video device, we assign it the symlink argus/cameraN
devices=$(ls /dev/video*)
counter=0
for device in $devices ; do
	path=$(udevadm info -a -p $(udevadm info -q path -n $device) | grep -m1 KERNELS== | sed -e 's/[ ]*KERNELS==//')
	echo "Found device $device with USB path $path. Assigning to ID $counter"
	echo KERNELS==$path, SUBSYSTEM==\"video4linux\", SYMLINK+=\"argus/camera$counter\", MODE="0666" >> $output_file
	((counter++))
done

echo "Created udev rules file $output_file"
