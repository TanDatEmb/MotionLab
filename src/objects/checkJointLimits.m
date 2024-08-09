%% Kiểm tra và giới hạn góc khớp
function jointAngles = checkJointLimits(jointAngles, jointLimits)
    for i = 1:length(jointAngles)
        jointAngles(i) = max(min(jointAngles(i), jointLimits(i, 2)), jointLimits(i, 1));
    end
end