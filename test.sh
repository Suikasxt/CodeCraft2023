if [ ! -n "$1" ] ;then
        map_number=1
else
        map_number=$1
fi
cd ./LinuxRelease
./Robot -f -d -m ./maps/2/$map_number.txt "../src/build/main $2"