#!/bin/bash

########################README#############################

#Place this file in the "server" folder and run it    

#with "sudo bash install.sh" command. After the 

#installing process is done, log out and log back in

#for the changes to take effect. The simulation.py can

#be excecuted from terminal using "python3 simulation.py" 

#command or in any other IDE, such as PyCharm. When the

#software is excecuted from IDE, make sure set the server

#fold as the "sources root" and use system default python 3.5+
 
#as the python interpreter. 

###############################################################

#install python 3.5+

sudo apt-get update -y

sudo apt-get install -y python3

#check python version

pythonVersionMajor=$(python3 -c "import sys; print(sys.version_info.major)")

pythonVersionMinor=$(python3 -c "import sys; print(sys.version_info.minor)")

if  (((pythonVersionMajor<3))||((pythonVersionMinor<5)))
then
	sudo apt-get update -y
	sudo apt-get install python3.5
fi

#install libspatialindex

sudo apt-get install -y curl

sudo apt-get install -y g++

sudo apt-get install -y make

curl -L http://download.osgeo.org/libspatialindex/spatialindex-src-1.8.5.tar.gz | tar xz

cd spatialindex-src-1.8.5

./configure

make

sudo make install

sudo ldconfig

cd ..

rm -rf spatialindex-src-1.8.5

#install pip
sudo apt-get install -y python3-pip

#install other python packages
sudo pip3 install pygame

sudo pip3 install networkx

sudo pip3 install shapely

sudo pip3 install rtree

sudo pip3 install numpy

sudo pip3 install scipy

sudo pip3 install google

sudo pip3 install matplotlib

sudo pip3 install protobuf

sudo apt-get install -y python3-tk

#set python path to software directory

echo export PYTHONPATH="$(pwd):$PYTHONPATH" >> ~/.bashrc
echo export PYTHONPATH="$(pwd):$PYTHONPATH" >> ~/.profile

#echo "import sys
#sys.path.append('$(pwd)')" | cat - ./core/simulation.py > temp && mv temp ./core/simulation.py

#cd core
#python3 simulation.py

