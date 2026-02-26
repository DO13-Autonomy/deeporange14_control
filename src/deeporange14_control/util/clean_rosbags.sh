# cleans rosbags from the specified temp directory
# this file should be run periodically to remove old or no-longer-needed files that may take up space

# hard-coded location and prefix for the rosbags recorded by the node (set in control.launch)
tmpfldr=/var/tmp
fileprefix=deeporange14_control

# get the list of bag files in the specified location
filelist=( $(find $tmpfldr -maxdepth 1 -name $fileprefix_*.bag) )

# if there are files found, list them and ask if the user wants to continue with removal
# if no files are found, inform and do nothing
if [ $filelist ];  then
    echo "Preparing to clean $fileprefix rosbags from $tmpfldr:"
    printf "%s\n" "${filelist[@]}"

    read -p "Continue with cleanup (all listed files will be removed) (Y/N): " choice

    # force the user to enter y/Y or n/N
    while [ "$choice" != "y" ] && [ "$choice" != "Y" ] && [ "$choice" != "n" ] && [ "$choice" != "N" ]; do
        echo "Please enter Y or N, or CTRL+C to cancel"
        read -p "Continue with cleanup (Y/N): " choice
    done

    if [ "$choice" == "y" ] || [ "$choice" == "Y" ]; then
        echo "Removing files"
        rm $tmpfldr/$fileprefix_*.bag
    else [ "$choice" == "n" ] || [ "$choice" == "N" ]
        echo "No files removed"
    fi
else
  echo "No $fileprefix rosbag files found in $tmpfldr"
fi