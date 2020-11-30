#!/bin/bash
# process raw data from capture
echo "Processing raw data from capture..."
# insert line break after each sample
sed 's/.\{124\}/&\n/g' capture.txt > samples.txt
# prompt to check for incomplete sampes
echo "Check samples.txt for incomplete or misaligned samples."
read -p "Press enter to continue..."
# separate raw bno055 data
sed 's/.\{48\}$//' samples.txt > bno055_raw.txt
# separate raw neo-m9n data
sed 's/.\{24\}$//' samples.txt > temp.txt
sed 's/^.\{76\}//' temp.txt > neom9n_raw.txt
# separate raw attitude kalman filter output data
sed 's/.\{8\}$//' samples.txt > temp.txt
sed 's/^.\{100\}//' temp.txt > attkf_raw.txt
# output number of processed samples
count=$(wc -l < samples.txt)
echo "Successfully processed ${count} samples."
# remove temporary files
rm temp.txt
