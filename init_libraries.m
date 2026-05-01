%% Add to Path Script

% Absolute path to the libraries on your system
if strcmp(getenv('USER'), 'lh')
    dynamixel_library_path = '/home/lh/nextcloud-sync/wpi/rbe502-control/project/DynamixelSDK';
else
    dynamixel_library_path = '/home/pinaka/DynamixelSDK';
end
% Add necessary folders and subfolders from the Dynamixel Library
addpath(genpath(dynamixel_library_path + "/c/include"));
addpath(dynamixel_library_path + "/c/build/linux64");
addpath(genpath(dynamixel_library_path + "/matlab"));
