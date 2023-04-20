if [ ! -n "$1" ] ;then
        map_number=1
else
        map_number=$1
fi
cd ./LinuxRelease
./Robot_gui -f -d -m ./maps/$map_number.txt "../src/build/main" "../modelpool/attack"