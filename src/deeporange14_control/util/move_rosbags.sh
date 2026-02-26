# interactively move rosbags from the specified temporary location to a local, non-temporary folder
# this will create a new folder if the destination does not exist
# note that there are no permission checks on the destination, so this will fail (with message: cannot
# move '<file>' to <dest>: Permission denied')
#
# this file is meant to move key rosbag files from the temporary location to a 'safe' location for
# further investigation or use; additional renaming may follow to better indicate conditions
# captured in the file, if desired

# hard-coded location and prefix for the rosbags recorded by the node (set in control.launch)
tmpfldr=/var/tmp
fileprefix=deeporange14_control

# get the list of bag files in the specified location
filelist=( $(find $tmpfldr -maxdepth 1 -name $fileprefix_*.bag) )

# if there are files found, list them and ask the user which files to move and to where
# if no files are found, inform and do nothing
if [ $filelist ];  then
    echo "List of $fileprefix rosbags found in $tmpfldr:"
    for f in "${!filelist[@]}"; do
        echo "[${f}]   ${filelist[$f]}"
    done

    read -p "Enter the indices of files to move: " -a choice

    printf "%s\n" ${choice[@]}

    read -p "Enter the destination for the files: " destfldr

    echo "Preparing to move specified $fileprefix rosbags from $tmpfldr to $destfldr"
    echo "Note that this script will fail if you do not have permission to write to $destfldr"

    if [ ! -d $destfldr ]; then
        echo "$destfldr does not exist, creating it"
        mkdir $destfldr
    fi

    for c in "${choice[@]}"; do
        if [ $c -lt ${#filelist[@]} ]; then
            destfile=$(echo ${filelist[$c]} | sed "s|$tmpfldr|$destfldr|")
            echo "Moving file ${filelist[$c]} to $destfile"
            mv ${filelist[$c]} ${destfile}
        else
            echo "Invalid index $c"
        fi
    done
else
  echo "No $fileprefix rosbag files found in $tmpfldr"
fi