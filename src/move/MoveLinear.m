classdef MoveLinear
    properties
        % Vị trí
        StartPoint
        EndPoint
        % Vận tốc
        StartVelocity
        DesiredVelocity
        EndVelocity
        % Vùng chuyển quỹ đạo
        SmoothZone

        % Tính toán
        DeltaTime
        TimeTrajectory
        PointTrajectory
        VelocityTrajectory
        AccelerationTrajectory
        OrientationTrajectory
        JerkTrajectory

        % Cài đặt
        PlanningMethod
        % Giới hạn
        MaxDistance
        MaxVelocity
        MaxAcceleration
        MinDeltaTime
        MaxJerk
    end

    methods
        %% Khởi tạo
        function obj = MoveLinear(startPoint, endPoint, startVel, desVel, endVel, zone)
            % Đọc thông số cấu hình
            config = readConfig('config.txt');
            % Cập nhật thuộc tính từ cấu hình
            obj.PlanningMethod = config('trajectoryPlanningMethod');

            obj.MaxDistance = str2double(config('maxDistance'));
            obj.MaxVelocity = str2double(config('maxVelocity'));
            obj.MaxAcceleration = str2double(config('maxAcceleration'));
            obj.MinDeltaTime = str2double(config('minDeltaTime'));
            obj.MaxJerk = str2double(config('maxJerk'));

            % Khởi tạo các thuộc tính
            obj.StartPoint = startPoint;
            obj.EndPoint = endPoint;

            obj.StartVelocity = startVel;
            obj.DesiredVelocity = desVel;
            obj.EndVelocity = endVel;

            obj.SmoothZone = zone;

            obj = obj.planTrajectory();
        end

        %% Trajectory Planning Method
        function obj = planTrajectory(obj)
            switch obj.PlanningMethod
                case 'LSPB'
                    obj = obj.lspbTrajectory();
                case 'Quintic'
                    obj = obj.quinticTrajectory();
                case 'JerkControl'
                    obj = obj.jerkControlTrajectory();
                otherwise
                    error('Unsupported planning method.');
            end
        end
        %% LSPB
        function obj = lspbTrajectory(obj)
            %% Các Thông Số Đầu Vào
            startPoint = obj.StartPoint.Position;
            endPoint = obj.EndPoint.Position;
            startQuaternion = obj.StartPoint.OrientationQuaternion;
            endQuaternion = obj.EndPoint.OrientationQuaternion;
            startVelocity = obj.StartVelocity;
            desiredVelocity = obj.DesiredVelocity;
            endVelocity = obj.EndVelocity;
            maxVelocity = obj.MaxVelocity;
            maxAcceleration = obj.MaxAcceleration;
            minDeltaTime = obj.MinDeltaTime;
            smoothZone = obj.SmoothZone;

           %% Các Thông Số Tính Toán
            % Độ dài quãng đường
            distance = norm(endPoint - startPoint);

            % Kiểm tra và điều chỉnh vận tốc nếu cần thiết
            if desiredVelocity > maxVelocity
                desiredVelocity = maxVelocity;
            end

            % Tính thời gian tăng tốc và giảm tốc
            timeAccel = (desiredVelocity - startVelocity) / maxAcceleration;
            timeDecel = (desiredVelocity - endVelocity) / maxAcceleration;

            if timeAccel < minDeltaTime || timeDecel < minDeltaTime
                error('Error: timeAccel or timeDecel is too small.');
            else
                disp('okey');
            end

            % Gia tốc thực tế
            actualAcceleration = (desiredVelocity - startVelocity) / timeAccel;
            actualDeceleration = (desiredVelocity - endVelocity) / timeDecel;

            % Tính quãng đường tăng tốc và giảm tốc
            distanceAccel = startVelocity * timeAccel + 0.5 * actualAcceleration * timeAccel^2;
            distanceDecel = desiredVelocity * timeDecel - 0.5 * actualDeceleration * timeDecel^2;

            % Tính quãng đường và thời gian vận tốc cố định
            distanceConstantVelocity = distance - (distanceAccel + distanceDecel);
            if distanceConstantVelocity < 0
                error('Error: distanceConstantVelocity is negative.');
            end

            timeConstantVelocity = distanceConstantVelocity / desiredVelocity;
            totalTime = timeAccel + timeConstantVelocity + timeDecel;

            % Số bước thời gian
            numStepsAccel = floor(timeAccel / minDeltaTime);
            numStepsConstantVelocity = floor(timeConstantVelocity / minDeltaTime);
            numStepsDecel = floor(timeDecel / minDeltaTime);

            %% Tổng hợp
            timeAccelArray = linspace(0, timeAccel, numStepsAccel);
            timeConstantVelocityArray = linspace(timeAccel, timeAccel + timeConstantVelocity, numStepsConstantVelocity);
            timeDecelArray = linspace(timeAccel + timeConstantVelocity, totalTime, numStepsDecel);

            timeVector = [timeAccelArray, timeConstantVelocityArray(2:end), timeDecelArray(2:end)];

            accelerationProfile = zeros(1, length(timeVector));
            velocityProfile = zeros(1, length(timeVector));
            pointProfile = zeros(1, length(timeVector));

            %% Tính toán các quỹ đạo
            for i = 2:length(timeVector)
                deltaTime = timeVector(i) - timeVector(i-1);
                if deltaTime < minDeltaTime
                    error('Error: deltaTime is too small.');
                end
                if timeVector(i) <= timeAccel
                    % Giai đoạn tăng tốc
                    accelerationProfile(i) = actualAcceleration;
                    velocityProfile(i) = startVelocity + actualAcceleration * timeVector(i);
                    pointProfile(i) = startVelocity * timeVector(i) + 0.5 * actualAcceleration * timeVector(i)^2;
                elseif timeVector(i) <= (timeAccel + timeConstantVelocity)
                    % Giai đoạn vận tốc cố định
                    accelerationProfile(i) = 0;
                    velocityProfile(i) = desiredVelocity;
                    pointProfile(i) = distanceAccel + desiredVelocity * (timeVector(i) - timeAccel);
                else
                    % Giai đoạn giảm tốc
                    accelerationProfile(i) = -actualDeceleration;
                    velocityProfile(i) = desiredVelocity - actualDeceleration * (timeVector(i) - timeAccel - timeConstantVelocity);
                    pointProfile(i) = distanceAccel + distanceConstantVelocity + desiredVelocity * (timeVector(i) - timeAccel - timeConstantVelocity) - 0.5 * actualDeceleration * (timeVector(i) - timeAccel - timeConstantVelocity)^2;
                end
            end
             
            %% Chuyển Đổi Sang Vector 3D
            direction = (endPoint - startPoint) / distance;
            pointProfile3D = startPoint + pointProfile' .* direction;
            velocityProfile3D = velocityProfile' .* direction;
            accelerationProfile3D = accelerationProfile' .* direction;

            %% Quy Hoạch Hướng Quaternions
            orientationTrajectory = slerpQuaternion(startQuaternion, endQuaternion, linspace(0, 1, length(timeVector)));

            %% Cập Nhật Các Thuộc Tính của Đối Tượng
            obj.TimeTrajectory = timeVector;
            obj.PointTrajectory = pointProfile3D;
            obj.VelocityTrajectory = velocityProfile3D;
            obj.AccelerationTrajectory = accelerationProfile3D;
            obj.OrientationTrajectory = orientationTrajectory;
        end
        %% Quintic
        function obj = quinticTrajectory(obj)
            %% Các Thông Số Đầu Vào
            startPoint = obj.StartPoint.Position;
            endPoint = obj.EndPoint.Position;
            startQuaternion = obj.StartPoint.OrientationQuaternion;
            endQuaternion = obj.EndPoint.OrientationQuaternion;
            startVelocity = obj.StartVelocity;
            desiredVelocity = obj.DesiredVelocity;
            endVelocity = obj.EndVelocity;
            maxVelocity = obj.MaxVelocity;
            maxAcceleration = obj.MaxAcceleration;
            minDeltaTime = obj.MinDeltaTime;

            %% Các Thông Số Tính Toán
            % Độ dài quãng đường
            distance = norm(endPoint - startPoint);

            % Kiểm tra và điều chỉnh vận tốc nếu cần thiết
            if desiredVelocity > maxVelocity
                desiredVelocity = maxVelocity;
            end

            % Tổng thời gian di chuyển
            totalTime = distance / desiredVelocity;

            % Kiểm tra và điều chỉnh thời gian di chuyển dựa trên gia tốc tối đa
            if totalTime < sqrt(distance / 4*maxAcceleration)
                totalTime = sqrt(distance / 4*maxAcceleration);
            end

            % Số bước thời gian
            numSteps = floor(totalTime / minDeltaTime);

            % Mảng thời gian
            timeVector = linspace(0, totalTime, numSteps);

            %% Tính Toán Trajectory với Quintic Polynomial
            % Hệ số cho đa thức bậc 5 (Quintic Polynomial)
            a0 = startPoint;
            a1 = startVelocity;
            a2 = 0;
            a3 = (10 * (endPoint - startPoint) - (6 * startVelocity + 4 * endVelocity) * totalTime) / (totalTime ^ 3);
            a4 = (-15 * (endPoint - startPoint) + (8 * startVelocity + 7 * endVelocity) * totalTime) / (totalTime ^ 4);
            a5 = (6 * (endPoint - startPoint) - (3 * startVelocity + 3 * endVelocity) * totalTime) / (totalTime ^ 5);

            % Vị trí, vận tốc và gia tốc tại mỗi bước thời gian
            pointProfile = zeros(numSteps, 3);
            velocityProfile = zeros(numSteps, 3);
            accelerationProfile = zeros(numSteps, 3);

            for i = 1:numSteps

                if i > 1
                    deltaTime = timeVector(i) - timeVector(i-1);
                    if deltaTime < minDeltaTime
                        error('Error: deltaTime');
                    end
                end

                t = timeVector(i);
                pointProfile(i, :) = a0 + a1 * t + a2 * t^2 + a3 * t^3 + a4 * t^4 + a5 * t^5;
                velocityProfile(i, :) = a1 + 2 * a2 * t + 3 * a3 * t^2 + 4 * a4 * t^3 + 5 * a5 * t^4;
                accelerationProfile(i, :) = 2 * a2 + 6 * a3 * t + 12 * a4 * t^2 + 20 * a5 * t^3;
                if norm(accelerationProfile(i, :)) > maxAcceleration
                    error('Error: accelerationProfile');
                end
            end

            %% Chuyển Đổi Sang Vector 3D
            direction = (endPoint - startPoint) / distance;
            pointProfile3D = pointProfile;
            velocityProfile3D = velocityProfile;
            accelerationProfile3D = accelerationProfile;

            %% Quy Hoạch Hướng Quaternions
            orientationTrajectory = slerpQuaternion(startQuaternion, endQuaternion, linspace(0, 1, length(timeVector)));

            %% Cập Nhật Các Thuộc Tính của Đối Tượng
            obj.TimeTrajectory = timeVector;
            obj.PointTrajectory = pointProfile3D;
            obj.VelocityTrajectory = velocityProfile3D;
            obj.AccelerationTrajectory = accelerationProfile3D;
            obj.OrientationTrajectory = orientationTrajectory;
        end
        %% Jerk
        function obj = jerkControlTrajectory(obj)
            %% Các Thông Số Đầu Vào
            startPoint = obj.StartPoint.Position;
            endPoint = obj.EndPoint.Position;
            startQuaternion = obj.StartPoint.OrientationQuaternion;
            endQuaternion = obj.EndPoint.OrientationQuaternion;
            startVelocity = obj.StartVelocity;
            desiredVelocity = obj.DesiredVelocity;
            endVelocity = obj.EndVelocity;
            maxVelocity = obj.MaxVelocity;
            maxAcceleration = obj.MaxAcceleration;
            maxJerk = obj.MaxJerk;
            minDeltaTime = obj.MinDeltaTime;

            %% Các Thông Số Tính Toán


            %% Chuyển Đổi Sang Vector 3D


            %% Quy Hoạch Hướng Quaternions
            orientationTrajectory = slerpQuaternion(startQuaternion, endQuaternion, linspace(0, 1, length(timeVector)));

            %% Cập Nhật Các Thuộc Tính của Đối Tượng
            obj.TimeTrajectory = timeVector;
            obj.PointTrajectory = pointProfile3D;
            obj.VelocityTrajectory = velocityProfile3D;
            obj.AccelerationTrajectory = accelerationProfile3D;
            obj.JerkTrajectory = jerkProfile3D;
            obj.OrientationTrajectory = orientationTrajectory;
        end
    end
end
