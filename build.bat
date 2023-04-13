@echo off
set build_dir=.\src\build

if exist %build_dir% (
    echo build~
) else (
    md %build_dir%
)
cmake -G "Kate" -B %build_dir% ./src
cmake --build ./src/build
