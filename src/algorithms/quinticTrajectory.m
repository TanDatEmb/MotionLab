function [posTraj, oriTraj, velTraj, accTraj] = quinticTrajectory(obj)
    t = linspace(0, obj.TimeSteps * obj.Dt, obj.TimeSteps);
    tf = obj.TimeSteps * obj.Dt;
    
    % Coefficients for the Quintic Polynomial
    a0 = obj.StartPosition;
    a1 = zeros(size(obj.StartPosition));
    a2 = zeros(size(obj.StartPosition));
    a3 = 10 * (obj.EndPosition - obj.StartPosition) / tf^3;
    a4 = -15 * (obj.EndPosition - obj.StartPosition) / tf^4;
    a5 = 6 * (obj.EndPosition - obj.StartPosition) / tf^5;
    
    posTraj = zeros(3, obj.TimeSteps);
    velTraj = zeros(3, obj.TimeSteps);
    accTraj = zeros(3, obj.TimeSteps);
    
    for i = 1:3
        posTraj(i, :) = a0(i) + a3(i) * t.^3 + a4(i) * t.^4 + a5(i) * t.^5;
        velTraj(i, :) = 3 * a3(i) * t.^2 + 4 * a4(i) * t.^3 + 5 * a5(i) * t.^4;
        accTraj(i, :) = 6 * a3(i) * t + 12 * a4(i) * t.^2 + 20 * a5(i) * t.^3;
    end
    
    % Slerp for orientation interpolation (quaternion)
    oriTraj = zeros(4, obj.TimeSteps);
    for i = 1:obj.TimeSteps
        ratio = t(i) / obj.Duration;
        oriTraj(:, i) = obj.slerp(obj.StartOrientation, obj.EndOrientation, ratio);
    end
end
