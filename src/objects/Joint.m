classdef Joint    
    properties
        Type          % Loại khớp (ví dụ: 'revolute' - quay, 'prismatic' - trượt, 'spherical' - cầu)
        Name          % Tên hoặc định danh của khớp
        Axis          % Trục quay hoặc dịch chuyển (đối với khớp quay hoặc trượt)
        Offset        % Độ lệch từ khớp trước đó trong cánh tay robot
        Limit         % Giới hạn khớp [min, max] cho khớp quay hoặc phạm vi cho khớp trượt
        JointVariable % Biến khớp hiện tại (ví dụ: góc cho khớp quay, dịch chuyển cho khớp trượt)
    end
    
    methods
        %% Khởi tạo đối tượng
        function obj = Joint(type, name, axis, offset, limit, jointVariable)
            if nargin > 0
                obj.Type = type;
                obj.Name = name;
                obj.Axis = axis;
                obj.Offset = offset;
                obj.Limit = limit;
                obj.JointVariable = jointVariable;
            end
        end
        %% Cập nhật biến khớp với kiểm tra hợp lệ
        function setJointVariable(obj, value) 
            if strcmp(obj.Type, 'revolute') && (value < obj.Limit(1) || value > obj.Limit(2))
                error('Biến khớp quay vượt quá giới hạn');
            elseif strcmp(obj.Type, 'prismatic') && (value < obj.Limit(1) || value > obj.Limit(2))
                error('Biến khớp trượt vượt quá giới hạn');
            end
            obj.JointVariable = value;
        end
        %% Lấy giá trị hiện tại của biến khớp
        function value = getJointVariable(obj)
            value = obj.JointVariable;
        end
        
        function [transformationMatrix] = computeTransformationMatrix(obj)
            if strcmp(obj.Type, 'revolute')
                % Ma trận quay cho khớp quay
                theta = obj.JointVariable;
                axis = obj.Axis;
                R = eye(3) + sin(theta) * skew(axis) + (1 - cos(theta)) * (skew(axis) * skew(axis));
                transformationMatrix = [R, obj.Offset; 0, 0, 0, 1];
                
            elseif strcmp(obj.Type, 'prismatic')
                % Ma trận dịch chuyển cho khớp trượt
                d = obj.JointVariable;
                axis = obj.Axis;
                T = eye(4);
                T(1:3, 4) = d * axis;
                transformationMatrix = T;
                
            elseif strcmp(obj.Type, 'spherical')
                % Chỗ để dành cho khớp cầu
                transformationMatrix = eye(4);
                
            else
                error('Loại khớp không được hỗ trợ');
            end
        end
        %% Tính ma trận đối xứng chéo cho một vector        
        function [skewMatrix] = skew(~, vector)

            skewMatrix = [ 0, -vector(3), vector(2); 
                           vector(3), 0, -vector(1); 
                           -vector(2), vector(1), 0];
        end
    end
end
