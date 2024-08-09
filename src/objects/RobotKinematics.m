classdef RobotKinematics
    properties
        RobotType % Loại robot
        Joints    % Mảng các đối tượng Joint
        DHParameters % Ma trận thông số DH
    end

    methods
        %% Khởi tạo
        function obj = RobotKinematics()
            % Đọc thông số cấu hình
            config = readConfig('config.txt');
            % Cập nhật thuộc tính từ cấu hình
            obj.RobotType = config('robotType');
            dhString = config('DHParameters');
            jointLimitString = config('jointLimit');
            jointTypesString = config('jointTypes');

            switch obj.RobotType
                case 'DH'
                    obj.DHParameters = str2num(dhString); %#ok<ST2NM>

                case '3_axis_CNC'
                    % Khởi tạo cho 3-axis CNC
                    jointLimitString = '0 1000; 0 1000; 0 1000';
                    jointTypesString = 'prismatic;prismatic;prismatic';

                    % Cập nhật lại file config.txt
                    updateConfig('temp\config.txt', 'jointLimit', jointLimitString);
                    updateConfig('temp\config.txt', 'jointTypes', jointTypesString);

                case '5_axis_CNC'
                    % Khởi tạo cho 5-axis CNC
                    jointLimitString = '0 1000; 0 1000; 0 1000,-90 90;-360 360';
                    jointTypesString = "prismatic;prismatic;prismatic;revolute;revolute";

                    % Cập nhật lại file config.txt
                    updateConfig('config.txt', 'jointLimit', jointLimitString);
                    updateConfig('config.txt', 'jointTypes', jointTypesString);

                otherwise
                    error('RobotType không hợp lệ hoặc không được hỗ trợ.');
            end

            % Tạo các đối tượng Joint từ cấu hình
            limits = str2num(jointLimitString); %#ok<ST2NM>
            jointTypes = strsplit(jointTypesString, ';');
            numJoints = length(jointTypes);
            obj.Joints = Joint.empty(numJoints, 0);
            for i = 1:numJoints
                % Tạo đối tượng Joint cho mỗi khớp
                obj.Joints(i) = Joint(jointTypes{i}, sprintf('Joint%d', i), [0; 0; 1], [0; 0; 0], limits(i, :), 0);
            end
        end
        %% Chọn cấu hình hoạt động
        function T = forwardKinematics(obj, jointValues)
            switch obj.RobotType
                case 'DH'
                    T = forwardKinematicsDH(obj,jointValues);
                case '3_axis_CNC'
                    T = forwardKinematics3AxisCNC(obj,jointValues);
                case '5_axis_CNC'
                    T = forwardKinematics5AxisCNC(obj,jointValues);
                otherwise
                    error('RobotType không hợp lệ hoặc không được hỗ trợ.');
            end
        end
        function T = inverseKinematics(obj, jointValues)
            switch obj.RobotType
                case 'DH'
                    T = inverseKinematicsDH(obj,jointValues);
                case '3_axis_CNC'
                    T = inverseKinematics3AxisCNC(obj,jointValues);
                case '5_axis_CNC'
                    T = inverseKinematics5AxisCNC(obj,jointValues);
                otherwise
                    error('RobotType không hợp lệ hoặc không được hỗ trợ.');
            end
        end

        %% Cấu hình tùy DH
        % Hàm giải động học thuận (Forward Kinematics)
        function T = forwardKinematicsDH(obj, jointValues)
            % Khởi tạo ma trận đồng nhất T
            T = eye(4);

            % Lấy số lượng khớp
            numJoints = size(obj.DHParameters, 1);

            % Tính toán động học thuận dựa trên thông số DH
            for i = 1:numJoints
                % Lấy thông số DH cho khớp hiện tại
                theta_0 = obj.DHParameters(i, 1);
                d_0 = obj.DHParameters(i, 2);
                a = obj.DHParameters(i, 3);
                alpha = obj.DHParameters(i, 4);
                jointValue = jointValues(i);

                % Xác định loại khớp từ thuộc tính Joint
                jointType = obj.Joints(i).Type;

                % Xử lý loại khớp bằng switch-case
                switch jointType
                    case 'revolute'
                        % Khớp quay: θ thay đổi theo biến khớp
                        theta = jointValue + theta_0;
                        d = d_0; % d không thay đổi

                    case 'prismatic'
                        % Khớp tịnh tiến: d thay đổi theo biến khớp
                        d = jointValue + d_0;
                        theta = theta_0; % θ không thay đổi

                    case 'fixed'
                        % Khớp cố định: sử dụng các giá trị gốc
                        theta = theta_0;
                        d = d_0;

                    otherwise
                        error('Loại khớp không hợp lệ: %s', jointType);
                end

                % Ma trận chuyển đổi DH
                A_i = [cos(theta), -sin(theta)*cos(alpha), sin(theta)*sin(alpha), a*cos(theta);
                    sin(theta), cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
                    0, sin(alpha), cos(alpha), d;
                    0, 0, 0, 1];

                % Nhân ma trận chuyển đổi
                T = T * A_i;
            end
        end

        % Hàm giải động học nghịch (Inverse Kinematics) sử dụng Gradient Descent
        function jointValues = inverseKinematicsDH(obj, desiredPose)
            % Số lượng khớp
            numJoints = length(obj.Joints);

            jointValues = zeros(numJoints, 1);
            for i = 1:numJoints
                jointValues(i) = obj.Joints(i).JointVariable; % Lấy giá trị khớp khởi tạo từ đối tượng Joint
            end
            maxIterations = 100; % Số lần lặp tối đa
            tolerance = 7e-6; % Ngưỡng sai số
            alpha = 1; % Tốc độ học ban đầu
            minAlpha = 0.01; % Tốc độ học tối thiểu
            decayRate = 0.99; % Tốc độ giảm của alpha

            % Vòng lặp hội tụ
            for iter = 1:maxIterations
                % Tính toán tư thế hiện tại từ biến khớp
                currentPose = obj.forwardKinematicsDH(jointValues);

                % Tính toán sai số giữa tư thế hiện tại và tư thế mong muốn
                errorPose = obj.computePoseError(desiredPose, currentPose);

                % Kiểm tra điều kiện dừng
                if norm(errorPose(:)) < tolerance
                    % disp(['Converged in ' num2str(iter) ' iterations.']);

                    % Cập nhật giá trị khớp trong đối tượng Joint
                    for i = 1:numJoints
                        obj.Joints(i).JointVariable = jointValues(i);
                    end

                    return;
                end

                % Tính toán Jacobian
                J = obj.computeJacobian(jointValues);

                % Tính toán gradient của hàm mục tiêu (sai số pose)
                gradient = J' * errorPose(:);

                % Cập nhật biến khớp
                jointValues = jointValues + alpha * gradient;

                for i = 1:numJoints
                    % Lưu giá trị gốc trước khi giới hạn
                    originalValue = jointValues(i);

                    % Giới hạn giá trị khớp trong phạm vi hợp lệ
                    jointValues(i) = min(max(jointValues(i), obj.Joints(i).Limit(1)), obj.Joints(i).Limit(2));

                    % Kiểm tra xem giá trị có bị thay đổi không, cho thấy giá trị đã chạm vào giới hạn
                    if jointValues(i) ~= originalValue
                        % Cảnh báo nếu giá trị khớp đã chạm vào giới hạn
                        error('Giá trị khớp %d %.6f đã chạm vào giới hạn (%.6f, %.6f).', i, originalValue, obj.Joints(i).Limit(1), obj.Joints(i).Limit(2));
                    end
                end

                % Điều chỉnh tốc độ học
                % alpha = max(alpha * decayRate, minAlpha);
            end

            error('Không hội tụ trong số lần lặp tối đa.');
        end

        % Tính toán Jacobian
        function J = computeJacobian(obj, jointValues)
            % Khởi tạo ma trận đồng nhất T
            T = eye(4);

            % Lấy số lượng khớp
            numJoints = length(jointValues);

            % Tính toán động học thuận dựa trên thông số DH
            for i = 1:numJoints
                % Lấy thông tin loại khớp từ lớp Joint
                jointType = obj.Joints(i).Type;

                % Tính toán Jacobian cho từng loại khớp
                switch jointType
                    case 'revolute'
                        % Khớp quay
                        z_i = T(1:3, 3);
                        p_i = T(1:3, 4) ; % - endEffectorPos
                        J(1:3, i) = cross(z_i, p_i); % Tốc độ thay đổi vị trí
                        J(4:6, i) = z_i;             % Tốc độ thay đổi định hướng

                    case 'prismatic'
                        % Khớp tịnh tiến
                        z_i = T(1:3, 3);
                        J(1:3, i) = z_i;            % Tốc độ thay đổi vị trí
                        J(4:6, i) = [0; 0; 0];      % Tốc độ thay đổi định hướng (không thay đổi)

                    case 'fixed'
                        J(1:3, i) = [0; 0; 0];
                        J(4:6, i) = [0; 0; 0];

                    case 'spherical'
                        % Khớp cầu: cần xử lý thêm nếu có yêu cầu đặc biệt
                        error('Khớp cầu chưa được triển khai.');

                    otherwise
                        error('Loại khớp không hợp lệ.');
                end

                % Lấy thông số DH cho khớp hiện tại
                theta_0 = obj.DHParameters(i, 1);
                d_0 = obj.DHParameters(i, 2);
                a = obj.DHParameters(i, 3);
                alpha = obj.DHParameters(i, 4);
                jointValue = jointValues(i);

                % Xác định loại khớp từ thuộc tính Joint
                jointType = obj.Joints(i).Type;

                % Xử lý loại khớp bằng switch-case
                switch jointType
                    case 'revolute'
                        % Khớp quay: θ thay đổi theo biến khớp
                        theta = jointValue + theta_0;
                        d = d_0; % d không thay đổi

                    case 'prismatic'
                        % Khớp tịnh tiến: d thay đổi theo biến khớp
                        d = jointValue + d_0;
                        theta = theta_0; % θ không thay đổi

                    case 'fixed'
                        % Khớp cố định: sử dụng các giá trị gốc
                        theta = theta_0;
                        d = d_0;

                    otherwise
                        error('Loại khớp không hợp lệ: %s', jointType);
                end

                % Ma trận chuyển đổi DH
                A_i = [cos(theta), -sin(theta)*cos(alpha), sin(theta)*sin(alpha), a*cos(theta);
                    sin(theta), cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
                    0, sin(alpha), cos(alpha), d;
                    0, 0, 0, 1];

                % Nhân ma trận chuyển đổi
                T = T * A_i;

            end
        end

        % Hàm tính sai số tư thế
        function errorPose = computePoseError(~, desiredPose, currentPose)
            % desiredPose và currentPose đều là ma trận đồng nhất 4x4
            % Tính toán sai số vị trí
            posError = desiredPose(1:3, 4) - currentPose(1:3, 4);

            % Tính toán sai số định hướng (hướng) sử dụng ma trận đồng nhất
            R_desired = desiredPose(1:3, 1:3);
            R_current = currentPose(1:3, 1:3);
            R_error = R_desired' * R_current; % Ma trận đồng nhất lỗi
            theta = 0.5 * (R_desired - R_current); % Sai số góc (sinh với góc nhỏ)

            % Chuyển đổi sai số góc thành vector
            angleError = [theta(3,2); theta(1,3); theta(2,1)]; % Vector sai số góc

            % Ghép nối sai số vị trí và hướng thành vector lỗi
            errorPose = [posError; angleError];
        end

        %% CNC 3 trục (3AxisCNC)
        % Hàm giải động học thuận (Forward Kinematics) cấu hình chuẩn CNC 3 trục (3AxisCNC)
        function T = forwardKinematics3AxisCNC(obj, jointValues)
            % Khởi tạo ma trận đồng nhất T
            T = [1 0 0 jointValues(1);
                0 1 0 jointValues(2);
                0 0 1 jointValues(3)];
        end
        % Hàm giải động học nghịch (Inverse Kinematics)
        function jointValues = inverseKinematics3AxisCNC(obj, desiredPose)
            jointValues(1) = desiredPose(1,4);
            jointValues(2) = desiredPose(2,4);
            jointValues(3) = desiredPose(3,4);
            for i = 1:3
                % Lưu giá trị gốc trước khi giới hạn
                originalValue = jointValues(i);

                % Giới hạn giá trị khớp trong phạm vi hợp lệ
                jointValues(i) = min(max(jointValues(i), obj.Joints(i).Limit(1)), obj.Joints(i).Limit(2));

                % Kiểm tra xem giá trị có bị thay đổi không, cho thấy giá trị đã chạm vào giới hạn
                if jointValues(i) ~= originalValue
                    % Cảnh báo nếu giá trị khớp đã chạm vào giới hạn
                    error('Giá trị khớp %d %.6f đã chạm vào giới hạn (%.6f, %.6f).', i, originalValue, obj.Joints(i).Limit(1), obj.Joints(i).Limit(2));
                end
            end
        end
        %% CNC 5 trục (5AxisCNC)

    end
end
