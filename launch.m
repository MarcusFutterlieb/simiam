function launch()

% Copyright (C) 2013 Georgia Tech Research Corporation
% see the LICENSE file included with this software

    clear;
    close all;
    bdclose all;
    clear all;
    clear fun;    
    clear java;
    clear classes;
    clc;

    if (isdeployed)
        [path, folder, ~] = fileparts(ctfroot);
        root_path = fullfile(path, folder);
    else
        root_path = fileparts(mfilename('fullpath'));
    end
    cd(root_path)% mfu_edit: helpful for enviroment pull down menue
    addpath(genpath(root_path));

    javaaddpath(fullfile(root_path, 'java'));

    app = simiam.ui.AppWindow(root_path, 'launcher');
    app.load_ui();

end
