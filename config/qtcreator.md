
Projects -> Build -> CMake -> CMAKE_BUILD_TYPE set to Debug --> Apply Configuration Changes

Projects -> Build -> Build Environment: add 
PATH  --> /opt/gstreamer/bin
LD_LIBRARY_PATH --> /opt/gstreamer/lib
PKG_CONFIG_PATH --> /opt/gstreamer/lib/pkgconfig


Projects -> Build -> Build steps -> details -> Tool arguments --> -j4

