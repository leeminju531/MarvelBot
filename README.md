# MarvelBot
marvelBot _ servingRobot
## install guide 
if u got serial error in building then, try

    sudo apt-get install ros-melodic-serial

and  'GeoGraphicLibError' 

    sudo apt-get install libgeographic-dev


## file guide 

    marvel Robot Configuration file : marvel_slam/launch/marvel_configuration.launch

this file describe that odom ~ base_scan transform , BaseControl , imu , ydlidar info

    marvel AMCL file : marvelbot_2dnav/launch/amcl.launch
this file describe that map to odom transform

