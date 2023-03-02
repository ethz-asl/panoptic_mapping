# Check target dir.
if [ -z $1 ]; 
then 
  echo "No target directory specified.";
  echo "Usage: ./download_flat_dataset.sh <target_dir>";
  exit 1
else 
  echo "Download the Flat Dataset to '$1'?"; 
  read -p "[Y/N]? " -n 1 -r
  if [[ ! $REPLY =~ ^[Yy]$ ]]
  then
    echo "Exiting."; 
    exit 1
  fi
fi

# Download.
echo "Download and processing the dataset. This may take some time..."; 
mkdir -p $1
wget http://robotics.ethz.ch/~asl-datasets/2021_Panoptic_Mapping/flat_dataset.zip -P $1
unzip $1/flat_dataset.zip -d $1
rm $1/flat_dataset.zip

# Finished.
echo "Successfully downloaded the 'flat_dataset' to '$1'.";
exit 0
