[ ! -d "./src/build" ] &&  mkdir ./src/build  
cd ./src/build
cmake ..
cmake --build .
