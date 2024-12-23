INSTALL_DIR=/home/zj/Desktop/wzr/sim_cpp/src/thirdparty

# INSTALL JSONCPP
JSONCPP_VERSION=1.9.4
JSONCPP_URL=https://github.com/open-source-parsers/jsoncpp.git
git clone $JSONCPP_URL
cd jsoncpp
git checkout $JSONCPP_VERSION
mkdir -p build
cd build
cmake -DCMAKE_INSTALL_PREFIX=$INSTALL_DIR ..
make -j8
make install