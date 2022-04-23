echo "(Re)installing Torcs..."
sudo rm -rf /usr/local/bin/torcs
sudo rm -rf /usr/local/lib/torcs
sudo rm -rf /usr/local/share/games/torcs

rm -rf /tmp/torcs-1.3.7

tar xjf torcs-1.3.7.tar.bz2 -C /tmp/

rm -r /tmp/torcs-1.3.7/src/drivers/berniw*
rm -r /tmp/torcs-1.3.7/src/drivers/bt
rm -r /tmp/torcs-1.3.7/src/drivers/damned
rm -r /tmp/torcs-1.3.7/src/drivers/inferno*
rm -r /tmp/torcs-1.3.7/src/drivers/lliaw*
rm -r /tmp/torcs-1.3.7/src/drivers/olethros
rm -r /tmp/torcs-1.3.7/src/drivers/sparkle
rm -r /tmp/torcs-1.3.7/src/drivers/tita

rm -r /tmp/torcs-1.3.7/export/drivers/berniw*
rm -r /tmp/torcs-1.3.7/export/drivers/bt
rm -r /tmp/torcs-1.3.7/export/drivers/damned
rm -r /tmp/torcs-1.3.7/export/drivers/inferno*
rm -r /tmp/torcs-1.3.7/export/drivers/lliaw*
rm -r /tmp/torcs-1.3.7/export/drivers/olethros
rm -r /tmp/torcs-1.3.7/export/drivers/sparkle
rm -r /tmp/torcs-1.3.7/export/drivers/tita

rm -r /tmp/torcs-1.3.7/data/tracks/a-speedway
rm -r /tmp/torcs-1.3.7/data/tracks/aalborg
rm -r /tmp/torcs-1.3.7/data/tracks/dirt*
rm -r /tmp/torcs-1.3.7/data/tracks/e-track*
rm -r /tmp/torcs-1.3.7/data/tracks/eroad
rm -r /tmp/torcs-1.3.7/data/tracks/g-track*
rm -r /tmp/torcs-1.3.7/data/tracks/mixed*
rm -r /tmp/torcs-1.3.7/data/tracks/road