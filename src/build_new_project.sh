#!/bin/bash

PROJ_DIR="$HOME/dev/projects"
#echo $PROJ_DIR

echo -n "Creating new project named "; echo -n $1; sleep 0.4; echo -n "."; sleep 0.4; echo -n "."; sleep 0.4; echo "."; sleep 0.4
echo -n "Building new dedicated workspace: "; echo $PROJ_DIR/$1
mkdir $PROJ_DIR/$1
echo "Moving into"
cd $PROJ_DIR/$1
echo "Building experimental playground"
mkdir playground
echo "Creating README file"
#touch README
echo "Writting into"
echo "README file, explained main features of the software. Also for git use" > README
{ echo -n "	***	"; echo -n $1; echo "	***"; } >> README
echo $2 >> README
echo "	-> building sources directory"
mkdir src
echo "	-> moving into"
cd src/
echo "	-> copying template makefile"
cp $HOME/dev/makefile .
echo "	-> building headers directory"
mkdir headers
echo "	-> building library directory"
mkdir libs
echo "	-> moving out"
cd ..
echo "Building data directory"
mkdir data
echo "Building config directory"
mkdir config
echo "Building doc directory"
mkdir doc
echo "Moving into"
cd doc/
echo "	-> building documentation directory"
mkdir documentation
echo "	-> building help directory"
mkdir tuto

echo "	-> Done"
echo -n "Project "; echo -n $1; echo " built and ready for great work !"
