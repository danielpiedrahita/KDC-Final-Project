%% Run Once to Initialize Everything
clear;
clc;

%Add everything to path
currentDir = fileparts(mfilename('fullpath'));
addpath(fullfile(currentDir, 'Scripts'));
addpath(fullfile(currentDir, 'CAD'));
addpath(genpath(fullfile(currentDir, 'Contact_Library')));

%Initialize Parameters
parameters_full();
contactmodelparams();
deriveDynamics();
beetleBot_DataFile();