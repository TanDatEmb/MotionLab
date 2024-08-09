classdef PointArray < handle
    properties
        Positions              % Mảng các vị trí [x, y, z] của các điểm
        OrientationMatrices    % Ma trận quay 3x3 cho từng điểm
        OrientationQuaternions % Quaternion quay cho từng điểm
        TransformationMatrices % Ma trận chuyển dời 4x4 cho từng điểm
        ReferenceFrame         % Hệ tọa độ tham chiếu chung
    end
    
    methods
        %% Khởi tạo đối tượng
        function obj = PointArray(positions, orientations, referenceFrame)
            if nargin ~= 0
                if size(positions, 2) ~= 3
                    error('Vị trí phải có kích thước N x 3.');
                end
                
                % Kiểm tra xem orientations có phải là ma trận quay hay quaternion
                if size(orientations, 2) == 4
                    % Nếu orientations có kích thước N x 4, giả định là quaternion
                    obj.OrientationQuaternions = quatnormalize(orientations);
                    obj.OrientationMatrices = arrayfun(@(i) quat2rotm(orientations(i,:)), 1:size(orientations, 1), 'UniformOutput', false);
                elseif size(orientations, 1) == 3 && size(orientations, 2) == 3
                    % Nếu orientations có kích thước 3 x 3, giả định là ma trận quay
                    obj.OrientationMatrices = orientations;
                    obj.OrientationQuaternions = arrayfun(@(i) rotm2quat(orientations(:,:,i)), 1:size(orientations, 3), 'UniformOutput', false);
                else
                    error('Dữ liệu orientations không hợp lệ.');
                end
                
                obj.Positions = positions;
                obj.ReferenceFrame = referenceFrame;
                
                % Tính ma trận chuyển dời cho từng điểm
                obj.TransformationMatrices = arrayfun(@(i) [obj.OrientationMatrices{i}, obj.Positions(i,:)'; 0 0 0 1], 1:size(positions, 1), 'UniformOutput', false);
            else
                % Khởi tạo mảng rỗng
                obj.Positions = [];
                obj.OrientationMatrices = {};
                obj.OrientationQuaternions = {};
                obj.TransformationMatrices = {};
                obj.ReferenceFrame = '';
            end
        end
        
        %% Cập nhật ma trận chuyển dời cho tất cả các điểm
        function obj = updateAllTransformationMatrices(obj)
            obj.TransformationMatrices = arrayfun(@(i) [obj.OrientationMatrices{i}, obj.Positions(i,:)'; 0 0 0 1], 1:size(obj.Positions, 1), 'UniformOutput', false);
        end
        
        %% Thêm điểm vào mảng
        function obj = addPoint(obj, position, orientation)
            if size(position, 2) ~= 3
                error('Vị trí phải có kích thước 1 x 3.');
            end
            
            % Kiểm tra xem orientation có phải là ma trận quay hay quaternion
            if numel(orientation) == 4
                % Nếu orientation là quaternion
                orientationQuaternion = quatnormalize(orientation);
                orientationMatrix = quat2rotm(orientation);
            elseif size(orientation, 1) == 3 && size(orientation, 2) == 3
                % Nếu orientation là ma trận quay
                orientationMatrix = orientation;
                orientationQuaternion = rotm2quat(orientation);
            else
                error('Dữ liệu orientation không hợp lệ.');
            end
            
            % Thêm thông tin vào mảng
            obj.Positions = [obj.Positions; position];
            obj.OrientationMatrices{end+1} = orientationMatrix;
            obj.OrientationQuaternions{end+1} = orientationQuaternion;
            obj.TransformationMatrices{end+1} = [orientationMatrix, position'; 0 0 0 1];
        end
        
        %% Xóa điểm tại chỉ số cụ thể
        function obj = removePoint(obj, index)
            if index > 0 && index <= size(obj.Positions, 1)
                obj.Positions(index, :) = [];
                obj.OrientationMatrices(index) = [];
                obj.OrientationQuaternions(index) = [];
                obj.TransformationMatrices(index) = [];
            else
                error('Chỉ số không hợp lệ.');
            end
        end
        
        %% Lấy thông tin của điểm tại chỉ số cụ thể
        function [position, orientationMatrix, orientationQuaternion, transformationMatrix] = getPoint(obj, index)
            if index > 0 && index <= size(obj.Positions, 1)
                position = obj.Positions(index, :);
                orientationMatrix = obj.OrientationMatrices{index};
                orientationQuaternion = obj.OrientationQuaternions{index};
                transformationMatrix = obj.TransformationMatrices{index};
            else
                error('Chỉ số không hợp lệ.');
            end
        end
    end
end
