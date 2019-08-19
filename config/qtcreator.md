If you openen this project before, remove `~code/patspc/CMakeList.user` or any other CMakeList.user files in this project
Open `~code/patspc/CMakeList` with qtcreator  
Delesect the imported  build, select the Desktop build. Change the paths of all builds to have the pc folder: pats/pc/build-*  

*It is imperitive that the pc folder is added in between the pats and build folder!*

Press Configure build button  
    
Additional settings:  
Left menu bar -> Projects -> Build -> CMake -> CMAKE_BUILD_TYPE set to Debug --> Apply Configuration Changes  
Left menu bar -> Projects -> Build -> Build steps -> details -> Tool arguments --> -j4  
Left menu bar -> Projects -> Build -> Clean steps ---> uncheck all and check clean  
Top Menubar -> Tools -> C++ -> Code Model -> Diagnostic Configurations: -> Clang-Tidy static analyzer checks  
Top Menubar -> Tools -> Text Editor -> Behavior -> Clean whitespace -> In entire document  
Top Menubar -> Tools -> Text Editor -> Behavior -> Tab size: -> 4  
Top Menubar -> Tools -> Text Editor -> Build & Run -> Save all files before build
