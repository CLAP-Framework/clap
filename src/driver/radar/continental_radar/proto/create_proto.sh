
protoc -I=./ --cpp_out=./build  *.proto
cp ./build/*.cc ../src/proto/
cp ./build/*.h ../include/continental_radar/proto/
