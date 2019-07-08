% John Kath
% Math 178 - Nonlinear Data Analytics
% Summer 2019
% Final Project Code - 7/7/19

%% process excel user data files in current directory

%
dirinfo = dir();
dirinfo(~[dirinfo.isdir]) = []; % remove non-directories
dirinfo = dirinfo(3:end); % remove . ..

% create directory for images
mkdir(pwd, 'images');

%
L = length(dirinfo);
for k = 1:L
    
    try
    
        % call imu analysis routines
        parentfolder = dirinfo(k).folder;
        subfolder = dirinfo(k).name;
        
        math178_workspace_data;
        math178_project_imu_orientation;
    
    catch e
        
        continue
        
    end
    
    % close all open figures
    close all
    
end

% fclose(fileID);
