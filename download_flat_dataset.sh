# Check target dir.
if [ -z $FLAT_DATA_DIR ]; 
then 
  echo "The download destination 'FLAT_DATA_DIR' variable is not set.";
  exit 1
else 
  echo "Download to '$FLAT_DATA_DIR'?"; 
  read -p "[Y/N]? " -n 1 -r
  if [[ ! $REPLY =~ ^[Yy]$ ]]
  then
      exit 1
  fi
fi

# Download.
mkdir -p $FLAT_DATA_DIR
wget http://robotics.ethz.ch/~asl-datasets/2021_Panoptic_Mapping/flat_dataset.zip -P $FLAT_DATA_DIR
unzip $FLAT_DATA_DIR/flat_dataset.zip -d $FLAT_DATA_DIR
rm $FLAT_DATA_DIR/flat_dataset.zip

# Finished.
echo "Successfully downloaded the 'flat_dataset' to '$FLAT_DATA_DIR'.";
exit 0
