function [position, velocity, acceleration] = lspb(startPos, endPos, timeSteps, maxVel)
    % Khởi tạo thời gian và các biến
    if isscalar(timeSteps)
        timeSteps = (0:timeSteps-1)';
    else
        timeSteps = timeSteps(:);
    end
    totalTime = max(timeSteps(:));
    
    % Tính toán tốc độ
    if nargin < 4
        maxVel = (endPos - startPos) / totalTime* 1.5;
    else
        maxVel = abs(maxVel) .* sign(endPos - startPos);
        if any(abs(maxVel) < abs(endPos - startPos) / totalTime)
            error('Tốc độ tối đa quá nhỏ');
        elseif any(abs(maxVel) > 2 * abs(endPos - startPos) / totalTime)
            error('Tốc độ tối đa quá lớn');
        end
    end
    
    % Xử lý trường hợp vị trí bắt đầu và kết thúc bằng nhau
    if all(startPos == endPos)
        position = ones(size(timeSteps)) .* startPos;
        velocity = zeros(size(timeSteps));
        acceleration = zeros(size(timeSteps));
        return;
    end
    
    % Tính toán thời gian chuyển tiếp và gia tốc
    blendTime = (startPos - endPos + maxVel .* totalTime) ./ maxVel;
    accelerationValue = maxVel ./ blendTime;
    
    % Khởi tạo vector vị trí, tốc độ và gia tốc
    position = zeros(length(timeSteps), numel(startPos));
    velocity = zeros(length(timeSteps), numel(startPos));
    acceleration = zeros(length(timeSteps), numel(startPos));
    
    % Tính toán các giá trị tại mỗi bước thời gian
    for i = 1:length(timeSteps)
        currentTime = timeSteps(i);
        for j = 1:numel(startPos)
            if currentTime <= blendTime(j)
                % Pha khởi đầu (Initial blend)
                position(i, j) = startPos(j) + 0.5 * accelerationValue(j) * currentTime^2;
                velocity(i, j) = accelerationValue(j) * currentTime;
                acceleration(i, j) = accelerationValue(j);
            elseif currentTime <= (totalTime - blendTime(j))
                % Pha chuyển động tuyến tính (Linear motion)
                position(i, j) = (endPos(j) + startPos(j) - maxVel(j) * totalTime) / 2 + maxVel(j) * currentTime;
                velocity(i, j) = maxVel(j);
                acceleration(i, j) = 0;
            else
                % Pha kết thúc (Final blend)
                position(i, j) = endPos(j) - 0.5 * accelerationValue(j) * totalTime^2 + accelerationValue(j) * totalTime * currentTime - 0.5 * accelerationValue(j) * currentTime^2;
                velocity(i, j) = accelerationValue(j) * totalTime - accelerationValue(j) * currentTime;
                acceleration(i, j) = -accelerationValue(j);
            end
        end
    end
    if nargout == 0
        plotTrajectory(timeSteps, position, velocity, acceleration);
    end
end

