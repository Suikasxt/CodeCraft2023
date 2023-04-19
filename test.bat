@echo off
set exe_dir=../src/build/Debug/main.exe
cd ./WindowsRelease
robot_gui.exe -f -d ..\modelpool\1.exe %exe_dir% -m maps\%1.txt
::robot_gui.exe -f  Demo\SimpleDemo.exe %exe_dir% -m maps\%1.txt
::robot_gui.exe -f  %exe_dir% Demo\SimpleDemo -m maps\%1.txt
::robot_gui.exe -f  %exe_dir% %exe_dir%  -m maps\%1.txt

cd ..