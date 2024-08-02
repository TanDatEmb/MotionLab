function [circleCenter, circleRadius, angleBetweenPoints, normalVector, isClockwise, isIntermediatePointOnArc] = circleFromThreePoints(startPos, endPos, midPos)
    % Tính vector giữa các điểm
    vector12 = midPos - startPos;
    vector13 = endPos - startPos;
    
    % Trung điểm của các đoạn thẳng
    midpoint12 = (startPos + midPos) / 2;
    midpoint13 = (startPos + endPos) / 2;
    
    % Vector pháp tuyến của mặt phẳng
    normalVector = cross(vector12, vector13);
    D = -dot(normalVector, startPos);
    
    % Tìm đường trung trực của các đoạn thẳng
    syms x y z
    eq1 = dot([x y z] - midpoint12, vector12) == 0;
    eq2 = dot([x y z] - midpoint13, vector13) == 0;
    
    % Giải hệ phương trình để tìm tâm đường tròn ngoại tiếp
    solution = solve([eq1, eq2, normalVector(1)*x + normalVector(2)*y + normalVector(3)*z + D == 0], [x, y, z]);
    circleCenter = double([solution.x, solution.y, solution.z]);
    
    % Tính bán kính đường tròn
    circleRadius = norm(circleCenter - startPos);
    
    % Tính góc giữa điểm 1 và điểm 3
    vectorCenterToStart = startPos - circleCenter;
    vectorCenterToEnd = endPos - circleCenter;
    cosTheta = dot(vectorCenterToStart, vectorCenterToEnd) / (norm(vectorCenterToStart) * norm(vectorCenterToEnd));
    angleBetweenPoints = acos(cosTheta);
    
    % Xác định chiều của đường đi (clockwise hay counterclockwise)
    isClockwise = dot(cross(vectorCenterToStart, vectorCenterToEnd), normalVector) > 0;
    
    % Kiểm tra điểm trung gian có nằm trên cung tròn không
    vectorIntermediateToCenter = midPos - circleCenter;
    cosThetaIntermediate = dot(vectorCenterToStart, vectorIntermediateToCenter) / (norm(vectorCenterToStart) * norm(vectorIntermediateToCenter));
    angleIntermediate = acos(cosThetaIntermediate);
    
    % Xác định chiều của góc giữa điểm đầu và điểm trung gian
    isIntermediateClockwise = dot(cross(vectorCenterToStart, vectorIntermediateToCenter), normalVector) > 0;
    
    % Kiểm tra nếu điểm trung gian nằm giữa điểm đầu và điểm cuối theo chiều của cung tròn
    if isClockwise
        isIntermediatePointOnArc = isIntermediateClockwise && (angleIntermediate <= angleBetweenPoints);
    else
        isIntermediatePointOnArc = ~isIntermediateClockwise && (angleIntermediate <= angleBetweenPoints);
        % Thay đổi góc để đi qua điểm trung gian
        angleBetweenPoints = angleBetweenPoints - 2*pi;
    end
end
