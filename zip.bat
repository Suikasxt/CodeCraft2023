@echo off
set zip_exe=bz.exe
set code_zip_dir=zips
if exist %code_zip_dir% (
    echo zipping~
) else (
    md %code_zip_dir%
)
set curtime=%time:~0,2%-%time:~3,2%

:delleft
if "%curtime:~0,1%"==" " set curtime=%curtime:~1%&&goto delleft

set filename=src-%curtime%.zip
echo %filename%
%zip_exe% c %code_zip_dir%/%filename% ./src/*.h ./src/*.cpp ./src/CMakeLists.txt