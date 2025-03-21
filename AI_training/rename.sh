#!/bin/bash

cd /media/DataPandemonium/chevan/dev/dataset/picar_VOC

function rename(){
  for f in *
  do
    name=$(echo $f | sed 's/_jpg.rf.*//' | sed 's/\.*//')
    extension=$(echo $f | sed 's/^.*\.//g')

    nameReplace=$(echo $f | sed 's/\.xml/.jpg/g')
    newName=""
    if [[ $name =~ ^[0-9]{4}$ ]]; then
      newName=$(echo "team45_${name}")
    else
      newName=$(echo "${name}")
    fi
    
    if [[ $extension == "xml" ]]; then
      sed -i "s/$nameReplace/${newName}.jpg/g" $f
    fi

    mv $f "${newName}.${extension}"
  done
}



cd "test/"
rename
cd ../train
rename
cd ../valid
rename
cd ..

mkdir images
mkdir annotations

find . -name "*.jpg" -exec mv {} images/ ';'
find . -name "*.xml" -exec mv {} annotations/ ';'
