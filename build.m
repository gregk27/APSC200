% Script to build java code and prepare matlab for use
% This script should be called at the start, followed by the desired imports
pwd;

% If ant is installed, then build the java code
if ~system("where ant");
    cd java;
    !ant clean

    % Clean class cache
    clear classes;

    !ant compile
    cd ..;
else
    warning("Ant not found, running from existing files")
    % Clean class cache
    clear classes;
end

% Add the java build folder
if exist("./java/build", 'dir') == 7
    javaaddpath("./java/build");
else
    error('Compiled files missing, cannot run')
end
