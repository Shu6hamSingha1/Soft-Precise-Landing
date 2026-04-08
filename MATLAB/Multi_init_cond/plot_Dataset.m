startTimestamp = datetime('2026-01-26 10:52:00');
% Specify the data folder path
dataFolder = 'Dataset'; 

% Get a list of all .mat files in the folder
dataFiles = dir(fullfile(dataFolder, '*.mat'));

% Check if there are any files
if isempty(dataFiles)
    error('No .mat files found in the specified folder.');
end

% Initialize an array to hold the names of files created after the timestamp
recentFiles = {};

% Loop through the files and check their modification dates
for k = 1:length(dataFiles)
    fileDate = datetime(dataFiles(k).date); % Get the modification date
    if fileDate > startTimestamp
        recentFiles{end+1} = dataFiles(k).name; % Store the file name
    end
end

% Check if any recent files were found
if isempty(recentFiles)
    fprintf('No files found after the specified timestamp.\n');
else
    % Load the recent files and plot the data
    for i = 1:length(recentFiles)
        filePath = fullfile(dataFolder, recentFiles{i});

        % clearvars -except dataFolder recentFiles i;
        loadedData = load(filePath);
        
        disp(recentFiles{i});

        % Call your plotting function
        plotter_adaptive(loadedData); % Assuming plotter_PID takes loaded data as input
        pause; % Pause for user interaction
    end
end
