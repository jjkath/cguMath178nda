% John Kath
% Math 178 - Nonlinear Data Analytics
% Summer 2019
% Final Project Code - 7/7/19

%% Obtain data for analysis

% clear workspace
% clearvars % -except an dt fn mn

% get user id and session
splitStr = strsplit(subfolder,'_');
userId = splitStr{1};
sessionNum = splitStr{3};

% get hmog user data
filepath = parentfolder + "/" + subfolder + "/";
% filepath = parentfolder + "\" + subfolder + "\";
act_data = readmatrix(filepath + 'Activity.csv');
acc_data = readmatrix(filepath + 'Accelerometer.csv');
gyr_data = readmatrix(filepath + 'Gyroscope.csv');
mag_data = readmatrix(filepath + 'Magnetometer.csv');


% activity label
string activityLabel;
switch act_data(1,9)
    case {1, 7, 13, 19}
        activityLabel = "Reading + Sitting";
    case {2, 8, 14, 20}
        activityLabel = "Reading + Walking";
    case {3, 9, 15, 21}
        activityLabel = "Writing + Sitting";
    case {4, 10, 16, 22}
        activityLabel = "Writing + Walking";
    case {5, 11, 17, 23}
        activityLabel = "Map + Sitting";
    case {6, 12, 18, 24}
        activityLabel = "Map + Walking";
    otherwise
        activityLabel = "";
end

% adjust array to uniform length
min_length = min([length(acc_data) length(gyr_data) length(mag_data)]);
acc_data = acc_data(1:min_length,4:6);
gyr_data = gyr_data(1:min_length,4:6);
mag_data = mag_data(1:min_length,4:6);

% clean up workspace
clear splitStr filepath min_length
