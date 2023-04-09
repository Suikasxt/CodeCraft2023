cd ./src/build
DEL Debug\main.exe /Q
cmake ..
cmake --build .
set seed=2023
cd ../../WindowsRelease
::Robot.exe -f -d -m ./maps/1.txt "../src/build/Debug/main.exe %1" -s %seed% 
::Robot.exe -f -d -m ./maps/2.txt "../src/build/Debug/main.exe %1" -s %seed%
Robot_gui.exe -f -d -m ./maps/6.txt "../src/build/Debug/main.exe %1" -s %seed%
::Robot.exe -d -f -m ./maps/1.txt "../src/build/Debug/main.exe %1" -s %seed%
::Robot.exe -d -f -m ./maps/2.txt "../src/build/Debug/main.exe %1" -s %seed%
::Robot.exe -d -f -m ./maps/3.txt "../src/build/Debug/main.exe %1" -s %seed%
::Robot.exe -d -f -m ./maps/4.txt "../src/build/Debug/main.exe %1" -s %seed%
cd ..