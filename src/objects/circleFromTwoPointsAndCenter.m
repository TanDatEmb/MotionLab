function [circleRadius, angleBetweenPoints, isClockwise] = circleFromTwoPointsAndCenter(startPoint, endPoint, circleCenter)
    % Extract positions from Point objects
    startPos = startPoint.Position;
    endPos = endPoint.Position;
    centerPos = circleCenter.Position;

    % Calculate the radius of the circle
    circleRadius = norm(startPos - centerPos);

    % Calculate the angle between the points
    vectorFromCenterToStart = startPos - centerPos;
    vectorFromCenterToEnd = endPos - centerPos;
    angleBetweenPoints = atan2(norm(cross(vectorFromCenterToStart, vectorFromCenterToEnd)), dot(vectorFromCenterToStart, vectorFromCenterToEnd));

    % Determine the direction (clockwise or counterclockwise)
    normalVector = cross(vectorFromCenterToStart, vectorFromCenterToEnd);
    if dot(normalVector, centerPos) > 0
        isClockwise = true;
    else
        isClockwise = false;
    end
end
