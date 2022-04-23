variable=`pwd`

sh deleteAndExtract.sh

cd $variable
sh generateRobot.sh "0"

cd /tmp/torcs-1.3.7
./configure
make
sudo make install
sudo make datainstall

cd $variable
sh includeMaps.sh

cd $variable
sh copyControllers.sh
