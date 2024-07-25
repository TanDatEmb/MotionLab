function [posTraj, oriTraj, velTraj, accTraj] = jerkControlTrajectory(obj)
    t = linspace(0, obj.TimeSteps * obj.Dt, obj.TimeSteps);
    tf = obj.TimeSteps * obj.Dt;
    
    % Coefficients for the Jerk Control Polynomial (seventh-order polynomial)
    a0 = obj.StartPosition;
    a1 = zeros(size(obj.StartPosition));
    a2 = zeros(size(obj.StartPosition));
    a3 = zeros(size(obj.StartPosition));
    a4 = 35 * (obj.EndPosition - obj.StartPosition) / tf^4;
    a5 = -84 * (obj.EndPosition - obj.StartPosition) / tf^5;
    a6 = 70 * (obj.EndPosition - obj.StartPosition) / tf^6;
    a7 = -20 * (obj.EndPosition - obj.StartPosition) / tf^7;
    
    posTraj = zeros(3, obj.TimeSteps);
    velTraj = zeros(3, obj.TimeSteps);
    accTraj = zeros(3, obj.TimeSteps);
    
    for i = 1:3
        posTraj(i, :) = a0(i) + a4(i) * t.^4 + a5(i) * t.^5 + a6(i) * t.^6 + a7(i) * t.^7;
        velTraj(i, :) = 4 * a4(i) * t.^3 + 5 * a5(i) * t.^4 + 6 * a6(i) * t.^5 + 7 * a7(i) * t.^6;
        accTraj(i, :) = 12 * a4(i) * t.^2 + 20 * a5(i) * t.^3 + 30 * a6(i) * t.^4 + 42 * a7(i) * t.^5;
    end
    
    % Slerp for orientation interpolation (quaternion)
    oriTraj = zeros(4, obj.TimeSteps);
    for i = 1:obj.TimeSteps
        ratio = t(i) / obj.Duration;
        oriTraj(:, i) = obj.slerp(obj.StartOrientation, obj.EndOrientation, ratio);
    end
end
