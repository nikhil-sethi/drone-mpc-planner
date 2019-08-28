If you openen this project before, remove `~code/patspc/CMakeList.user` or any other CMakeList.user files in this project
Open `~code/patspc/CMakeList` with qtcreator  
Delesect the imported  build, select the Desktop build. Change the paths of all builds to have the pc folder: pats/pc/build-*  

*It is imperitive that the pc folder is added in between the pats and build folder!*

Press Configure build button  
    
Additional settings:  
Left menu bar -> Projects -> Build -> Edit build configuration dropdown: Make sure the debug configuration is selected.
Left menu bar -> Projects -> Build -> Build steps -> details -> Tool arguments --> -j4  
Left menu bar -> Projects -> Build -> Clean steps ---> uncheck all and check clean  
Top Menubar -> Tools -> Option -> C++ -> Code Model -> Diagnostic Configurations: -> Clang-Tidy static analyzer checks  
Top Menubar -> Tools -> Option -> Text Editor -> Behavior -> Clean whitespace -> In entire document  
Top Menubar -> Tools -> Option -> Text Editor -> Behavior -> Tab size: -> 4  
Top Menubar -> Tools -> Option -> Text Editor -> Build & Run -> Save all files before build
Top Menubar -> Tools -> Option -> C++ -> Code Model -> Edit -> Tab policy: Spaces only
Top Menubar -> Tools -> Option -> C++ -> Code Model -> Edit -> Tab size:: 4
Top Menubar -> Tools -> Option -> C++ -> Code Model -> Edit -> Indent size:: 4
