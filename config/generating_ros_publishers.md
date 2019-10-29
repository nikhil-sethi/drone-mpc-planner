## creating dds messages from idl files using eProsima fastrtpsgen
An incomprehensive guide

#create folder with the idl file of the message you want to create. 
mkdir ~/idl_files

#copy idl files from ros
cp /opt/ros/dashing/share/nav_msgs/msg/Path.idl ~/idl_files/

#download and compile fastrtps
eProsima_FastRTPS-1.7.2-Linux

#run fastrtps like this:
*your_download_folder*/eProsima_FastRTPS-1.7.2-Linux/bin/fastrtpsgen  -example CMake Path.idl

#you will find out about missing dependencies. Add these folders of these dependencies to your folder and add symlinks in the subfolders if neccesary
cp /opt/ros/dashing/share/std_msgs ~/idl_files/
cp /opt/ros/dashing/share/builtin_interfaces ~/idl_files/
cd std_msgs/msg
ln -s ../../builtin_interfaces builtin_interfaces

#if you run into this "error: 'int32' was not defined previously" then replace all int32 with long and uint32 with unsigned long etc.	

#solve these "warning: missing terminating ..."

#if you run into "error: was redefined" remove double header inclusions

