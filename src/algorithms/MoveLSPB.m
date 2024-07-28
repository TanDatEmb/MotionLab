function [timeVector, positionProfile3D, velocityProfile3D, accelerationProfile3D] = MoveLSPB(startPosition, endPosition, maxVelocity, maxAcceleration, resolution, minDt)
   
    %% Các Thông Số Tính Toán
    % Độ dài quãng đường
    distance = norm(endPosition - startPosition);

    % Thời gian tăng giảm tốc tối đa
    timeAccelMax = maxVelocity / maxAcceleration;

    % Số bước thời gian tối đa cho tăng giảm tốc
    numStepsAccelMax = ceil(timeAccelMax / minDt);

    % Gia tốc thực tế
    actualAcceleration = maxVelocity / timeAccelMax;

    % Tích phân gia tốc thực tế theo thời gian tối ưu để tính quãng đường tăng giảm tốc
    distanceAccel = 0.5 * actualAcceleration * timeAccelMax^2;

    % Thời gian vận tốc cố định
    distanceConstantVelocity = distance - 2 * distanceAccel;
    timeConstantVelocity = distanceConstantVelocity / maxVelocity;
    totalTime = 2 * timeAccelMax + timeConstantVelocity;

    % Số bước thời gian tối đa cho vận tốc cố định
    numStepsConstantVelocity = ceil(timeConstantVelocity / minDt);

    %% 
    timeAccel = linspace(0, timeAccelMax, numStepsAccelMax);
    timeConstantVelocityArray = linspace(timeAccelMax, timeAccelMax + timeConstantVelocity, numStepsConstantVelocity);
    timeDecel = linspace(timeAccelMax + timeConstantVelocity, 2 * timeAccelMax + timeConstantVelocity, numStepsAccelMax);

    timeVector = [timeAccel, timeConstantVelocityArray(2:end), timeDecel(2:end)];

    accelerationProfile = zeros(1, length(timeVector));
    velocityProfile = zeros(1, length(timeVector));
    positionProfile = zeros(1, length(timeVector));

    % Gia tốc tăng tốc
    accelerationProfile(1:numStepsAccelMax) = actualAcceleration;

    % Gia tốc giữ tốc độ cố định
    accelerationProfile(numStepsAccelMax+1:numStepsAccelMax+numStepsConstantVelocity-1) = 0;

    % Gia tốc giảm tốc
    accelerationProfile(numStepsAccelMax+numStepsConstantVelocity:end) = -actualAcceleration;

    % Tính vận tốc và vị trí
    for i = 2:length(timeVector)
        deltaTime = timeVector(i) - timeVector(i-1);
        velocityProfile(i) = velocityProfile(i-1) + accelerationProfile(i-1) * deltaTime;
        positionProfile(i) = positionProfile(i-1) + velocityProfile(i-1) * deltaTime;
    end

    %% Chuyển Đổi Sang Vector 3D
    direction = (endPosition - startPosition) / distance;
    positionProfile3D = startPosition + positionProfile' .* direction;
    velocityProfile3D = velocityProfile' .* direction;
    accelerationProfile3D = accelerationProfile' .* direction;
end
