function plot3DTrajectoryWithQuaternions(position, quaternions)
    % Kiểm tra kích thước dữ liệu
    if size(position, 2) ~= 3 || size(quaternions, 2) ~= 4
        error('Position data must be Nx3 and quaternion data must be Nx4');
    end
    
    % Số lượng điểm dữ liệu
    numPoints = size(position, 1);
    
    % Thiết lập figure
    figure;
    hold on;
    
    % Vẽ đường đi của vị trí
    plot3(position(:, 1), position(:, 2), position(:, 3), 'b');
    xlabel('X Position');
    ylabel('Y Position');
    zlabel('Z Position');
    title('3D Position Trajectory with Orientation');
    grid on;
    axis equal;
    
    % Đặt chiều dài của các trục tọa độ
    axisLength = 0.1;
    
    % Vẽ hệ trục tọa độ tại mỗi vị trí sử dụng plotTransforms
    for i = 1:numPoints
        % Lấy vị trí và quaternion tại điểm hiện tại
        pos = position(i, :);
        quat = quaternions(i, :);
        
        % Vẽ hệ trục tọa độ tại vị trí hiện tại với quaternion
        plotTransforms(pos, quat, 'FrameSize', axisLength);
    end
    
    hold off;
end
