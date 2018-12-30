Remove pc/CMakeList.user  
Open pc/CMakeList.txt with qtcreator  
Delesect the imported  build, select the Desktop build. Change the paths of all builds to have the pc folder: pats/pc/build-*  
Press Configure build button  
  
Cmake will fail, perform the following steps to fix:
  
Projects -> Build -> Build Environment: add  
PATH  --> /opt/gstreamer/bin  
LD_LIBRARY_PATH --> /opt/gstreamer/lib  
PKG_CONFIG_PATH --> /opt/gstreamer/lib/pkgconfig  
  
Additional settings:  
Projects -> Build -> CMake -> CMAKE_BUILD_TYPE set to Debug --> Apply Configuration Changes  
Projects -> Build -> Build steps -> details -> Tool arguments --> -j4  
Projects -> Build -> Clean steps ---> uncheck all and check clean  

