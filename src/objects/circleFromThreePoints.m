function [circleCenter, circleRadius, angleBetweenPoints, isClockwise, isIntermediatePointOnArc] = circleFromThreePoints(startPoint, midPoint, endPoint)
    % Tính vector giữa các điểm
    vector12 = midPoint.Position - startPoint.Position;
    vector13 = endPoint.Position - startPoint.Position;
    
    % Trung điểm của các đoạn thẳng
    midpoint12 = (startPoint.Position + midPoint.Position) / 2;
    midpoint13 = (startPoint.Position + endPoint.Position) / 2;
    
    % Vector pháp tuyến của mặt phẳng
    normalVector = cross(vector12, vector13);
    D = -dot(normalVector, startPoint.Position);
    
    % Tìm đường trung trực của các đoạn thẳng
    syms x y z
    eq1 = dot([x y z] - midpoint12, vector12) == 0;
    eq2 = dot([x y z] - midpoint13, vector13) == 0;
    
    % Giải hệ phương trình để tìm tâm đường tròn ngoại tiếp
    solution = solve([eq1, eq2, normalVector(1)*x + normalVector(2)*y + normalVector(3)*z + D == 0], [x, y, z]);
    circleCenter = double([solution.x, solution.y, solution.z]);
    
    % Tính bán kính đường tròn
    circleRadius = norm(circleCenter - startPoint.Position);
    
    % Tính góc giữa điểm 1 và điểm 3
    vectorStartToCenter = startPoint.Position - circleCenter;
    vectorEndToCenter = endPoint.Position - circleCenter;
    cosTheta = dot(vectorStartToCenter, vectorEndToCenter) / (norm(vectorStartToCenter) * norm(vectorEndToCenter));
    angleBetweenPoints = acos(cosTheta);
    
    % Xác định chiều của đường đi (clockwise hay counterclockwise)
    isClockwise = dot(cross(vectorStartToCenter, vectorEndToCenter), normalVector) > 0;
    
    % Kiểm tra điểm trung gian có nằm trên cung tròn không
    vectorIntermediateToCenter = midPoint.Position - circleCenter;
    cosThetaIntermediate = dot(vectorStartToCenter, vectorIntermediateToCenter) / (norm(vectorStartToCenter) * norm(vectorIntermediateToCenter));
    angleIntermediate = acos(cosThetaIntermediate);
    
    % Xác định chiều của góc giữa điểm đầu và điểm trung gian
    isIntermediateClockwise = dot(cross(vectorStartToCenter, vectorIntermediateToCenter), normalVector) > 0;
    
    % Kiểm tra nếu điểm trung gian nằm giữa điểm đầu và điểm cuối theo chiều của cung tròn
    if isClockwise
        isIntermediatePointOnArc = isIntermediateClockwise && (angleIntermediate <= angleBetweenPoints);
    else
        isIntermediatePointOnArc = ~isIntermediateClockwise && (angleIntermediate <= angleBetweenPoints);
        % Thay đổi góc để đi qua điểm trung gian
        angleBetweenPoints = angleBetweenPoints - 2*pi;
    end
end
