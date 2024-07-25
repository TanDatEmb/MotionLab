classdef MoveLinear
    properties
        % Vị trí
        StartPosition
        EndPosition
        % Vận tốc
        StartVelocity
        DesiredVelocity
        EndVelocity
        % Vùng chuyển quỹ đạo
        SmoothZone

        % Tính toán
        DeltaTime
        TimeTrajectory
        PositionTrajectory
        VelocityTrajectory
        AccelerationTrajectory
        OrientationTrajectory

        %Cài đặt
        PlanningMethod
        % Giới hạn
        MaxDistance
        MaxVelocity
        MaxAcceleration
        MinDeltaTime
    end

    methods
        %% Khởi tạo
        function obj = MoveLinear(startPos, endPos, startVel, desVel, endVel, zone)

            % Đọc thông số cấu hình
            config = readConfig('config.txt');
            % Cập nhật thuộc tính từ cấu hình
            obj.PlanningMethod = config('trajectoryPlanningMethod');

            obj.MaxDistance = str2double(config('maxDistance'));
            obj.MaxVelocity = str2double(config('maxVelocity'));
            obj.MaxAcceleration = str2double(config('maxAcceleration'));
            obj.MinDeltaTime = str2double(config('minDeltaTime'));

            % Khởi tạo các thuộc tính
            obj.StartPosition = startPos;
            obj.EndPosition = endPos;

            obj.StartVelocity = startVel;
            obj.DesiredVelocity = desVel;
            obj.EndVelocity = endVel;

            obj.SmoothZone = zone;

            [obj.PositionTrajectory, obj.OrientationTrajectory, obj.VelocityTrajectory, obj.AccelerationTrajectory] = obj.planTrajectory();
        end

        %% Trajectory Planning Method
        function [posTraj, oriTraj, velTraj, accTraj] = planTrajectory(obj)
            switch obj.PlanningMethod
                case 'LSPB'
                    [posTraj, oriTraj, velTraj, accTraj] = lspbTrajectory(obj);
                case 'Quintic'
                    [posTraj, oriTraj, velTraj, accTraj] = quinticTrajectory(obj);
                case 'JerkControl'
                    [posTraj, oriTraj, velTraj, accTraj] = jerkControlTrajectory(obj);
                otherwise
                    error('Unsupported planning method.');
            end
        end
    end
end
