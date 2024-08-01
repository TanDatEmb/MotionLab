function [circleRadius, angleBetweenPoints, isClockwise] = circleFromTwoPointsAndCenter(startPoint, endPoint, circleCenter)
    % Tính bán kính của đường tròn
    circleRadius = norm(startPoint.Position - circleCenter.Position);
    
    % Tính góc giữa hai điểm (clockwise hay counterclockwise)
    vectorStartToCenter = startPoint.Position - circleCenter.Position;
    vectorEndToCenter = endPoint.Position - circleCenter.Position;
    
    cosTheta = dot(vectorStartToCenter, vectorEndToCenter) / (norm(vectorStartToCenter) * norm(vectorEndToCenter));
    angleBetweenPoints = acos(cosTheta);
    
    % Vector pháp tuyến của mặt phẳng
    normalVector = cross(vectorStartToCenter, vectorEndToCenter);

    % Xác định chiều của đường đi (clockwise hay counterclockwise)
    isClockwise = dot(cross(vectorStartToCenter, vectorEndToCenter), normalVector) > 0;
    if isClockwise
        % do something
    else
        % do something
    end

end
