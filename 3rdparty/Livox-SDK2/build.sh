BUILD_DIR=build
if [ -d $BUILD ]; then
  rm -rf $BUILD
fi
mkdir $BUILD && cd $BUILD
cmake ..
make
