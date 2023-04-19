[ ! -d "./src/build" ] &&  mkdir ./src/build  
cd ./src/build
rm * -rf
cmake ..
cmake --build .
