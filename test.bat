cd ./src/build
cmake ..
cmake --build .
cd ../../WindowsRelease
Robot.exe -f -d -m ./maps/1.txt "../src/build/Debug/main.exe"
Robot.exe -f -d -m ./maps/2.txt "../src/build/Debug/main.exe"
Robot.exe -f -d -m ./maps/3.txt "../src/build/Debug/main.exe"
Robot.exe -f -d -m ./maps/4.txt "../src/build/Debug/main.exe"
Robot_gui.exe -d -m ./maps/3.txt "../src/build/Debug/main.exe"
cd ..