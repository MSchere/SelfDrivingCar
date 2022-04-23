variable=`pwd`

sudo rm /usr/local/lib/torcs/drivers/Robot$1/Robot$1.so
rm -r /tmp/torcs-1.3.7/src/drivers/Robot$1
cd /tmp/torcs-1.3.7
./robotgen -n "Robot"$1 -a "INVETT-Car" -c "car5-trb1" -gpl

cd /tmp/torcs-1.3.7/src/drivers/Robot$1/
rm Robot$1.cpp
rm Makefile

ln -s $variable/files/code/makes/Makefile$1 ./Makefile
ln -s $variable/files/code/src/* .
ln -s $variable/files/code/h/* .

cd $variable

