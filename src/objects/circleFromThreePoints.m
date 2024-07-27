function [circleCenter, circleRadius, angleBetweenPoints, isClockwise] = circleFromThreePoints(point1, point2, point3)
    % Tính vector giữa các điểm
    vector12 = point2.Position - point1.Position;
    vector13 = point3.Position - point1.Position;

    % Trung điểm của các đoạn thẳng
    midpoint12 = (point1.Position + point2.Position) / 2;
    midpoint13 = (point1.Position + point3.Position) / 2;

    % Vector pháp tuyến của mặt phẳng
    normalVector = cross(vector12, vector13);
    D = -dot(normalVector, point1.Position);

    % Tìm đường trung trực của các đoạn thẳng
    syms x y z
    eq1 = dot([x y z] - midpoint12, vector12) == 0;
    eq2 = dot([x y z] - midpoint13, vector13) == 0;

    % Giải hệ phương trình để tìm tâm đường tròn ngoại tiếp
    solution = solve([eq1, eq2, normalVector(1)*x + normalVector(2)*y + normalVector(3)*z + D == 0], [x, y, z]);
    circleCenter = double([solution.x, solution.y, solution.z]);

    % Tính bán kính đường tròn ngoại tiếp
    circleRadius = norm(circleCenter - point1.Position);

    % Tính góc giữa điểm 1 và điểm 3
    vectorStartToCenter = point1.Position - circleCenter;
    vectorEndToCenter = point3.Position - circleCenter;
    cosTheta = dot(vectorStartToCenter, vectorEndToCenter) / (norm(vectorStartToCenter) * norm(vectorEndToCenter));
    angleBetweenPoints = acosd(cosTheta);

    % Xác định chiều của đường đi (clockwise hay counterclockwise)
isClockwise = dot(cross(vector12, vector13), normalVector) > 0;
end

