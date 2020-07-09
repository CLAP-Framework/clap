
#protoc -I=./ --cpp_out=./build  *.proto
protoc -I=./ --cpp_out=./build  srr_radar.proto
#cp ./build/*.cc ../src/proto/
#cp ./build/*.h ../include/srr_radar/proto/
