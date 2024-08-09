function saveJointValues(pointsArray)
    robot = RobotKinematics();
    % Đọc thông số cấu hình
    config = readConfig('config.txt');
    jointTypesString = config('jointTypes');
    jointTypes = strsplit(jointTypesString, ';');

    outputFilename = 'joint_values.txt';
    
    % Open a file in the data directory for writing
    filepath = fullfile('data', outputFilename);
    fileID = fopen(filepath, 'w');

    % Get the current date and time
    currentDateTime = datetime("now");
    
    % Write the date and time at the beginning of the file
    fprintf(fileID, 'Date and Time: %s\n', currentDateTime);
    fprintf(fileID, '---------------------------------\n');

    % Loop through each transformation matrix in the pointsArray
    for i = 1:numel(pointsArray.TransformationMatrices)
        % Calculate joint angles using inverse kinematics
        jointValue = robot.inverseKinematics(pointsArray.TransformationMatrices{i});
    
        % Write only non-fixed joint values to the file, separated by semicolons
        fprintf(fileID, 'N%d; ', i);
        for j = 1:length(jointTypes)
            if ~strcmp(jointTypes{j}, 'fixed')
                fprintf(fileID, '%.6f; ', jointValue(j));
            end
        end
        fprintf(fileID, '\n'); % New line for each set of joint values
    end
    
    % Close the file
    fclose(fileID);
    
    % disp(['Joint values saved to ' filepath]);
end
