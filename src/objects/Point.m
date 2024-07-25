classdef Point
    properties
        Position
        OrientationMatrix
        OrientationQuaternion
        TransformationMatrix
        ReferenceFrame
    end

    methods
        %% Khởi tạo đối tượng
        function obj = Point(position, orientation, referenceFrame)
            if nargin ~= 0
                obj.Position = position;
                obj.ReferenceFrame = referenceFrame;
                % Tạo điểm bằng khai báo quaternions
                if isnumeric(orientation) && numel(orientation) == 4
                    obj.OrientationQuaternion = orientation;
                    obj.OrientationMatrix = quat2rotm(orientation);
                else % Tạo điểm bằng khai báo matrix
                    obj.OrientationMatrix = orientation;
                    obj.OrientationQuaternion = rotm2quat(orientation);
                end
            else
                % Tạo điểm gốc tọa độ 0
                obj.Position = [0 0 0];
                obj.OrientationMatrix = eye(3);
                obj.OrientationQuaternion = quaternion([1 0 0 0]);
                obj.ReferenceFrame = ''; % Hệ World
            end
            % Tính ma trận chuyển dời
            obj.TransformationMatrix = [obj.OrientationMatrix, obj.Position'; 0 0 0 1];
        end
    end
end