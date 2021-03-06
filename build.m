% Script to build java code and prepare matlab for use

pwd;
cd java;
!ant clean

% Clean class cache
clear classes;

!ant compile
cd ..;

% Add the java build folder
javaaddpath("./java/build");