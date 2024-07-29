function plotTrajectory(position, velocity, acceleration, quaternions, timeSteps)
    numDimensions = size(position, 2);
    
    figure;
    
    if numDimensions == 1
        subplot(4,1,1);
        plot(timeSteps, position);
        xlabel('Time');
        ylabel('Position');
        title('1D Position Trajectory');
        
        subplot(4,1,2);
        plot(timeSteps, velocity);
        xlabel('Time');
        ylabel('Velocity');
        title('1D Velocity Trajectory');
        
        subplot(4,1,3);
        plot(timeSteps, acceleration);
        xlabel('Time');
        ylabel('Acceleration');
        title('1D Acceleration Trajectory');
    elseif numDimensions == 2
        subplot(4,1,1);
        plot(timeSteps, position(:, 1), 'r', timeSteps, position(:, 2), 'b');
        xlabel('Time');
        ylabel('Position');
        legend('X Position', 'Y Position');
        title('2D Position Trajectory');
        
        subplot(4,1,2);
        plot(timeSteps, velocity(:, 1), 'r', timeSteps, velocity(:, 2), 'b');
        xlabel('Time');
        ylabel('Velocity');
        legend('X Velocity', 'Y Velocity');
        title('2D Velocity Trajectory');
        
        subplot(4,1,3);
        plot(timeSteps, acceleration(:, 1), 'r', timeSteps, acceleration(:, 2), 'b');
        xlabel('Time');
        ylabel('Acceleration');
        legend('X Acceleration', 'Y Acceleration');
        title('2D Acceleration Trajectory');
    elseif numDimensions == 3
        subplot(5,1,1);
        plot(timeSteps, position(:, 1), 'r', timeSteps, position(:, 2), 'g', timeSteps, position(:, 3), 'b');
        xlabel('Time');
        ylabel('Position');
        legend('X Position', 'Y Position', 'Z Position');
        title('Position vs Time');
        
        subplot(5,1,2);
        plot(timeSteps, velocity(:, 1), 'r', timeSteps, velocity(:, 2), 'g', timeSteps, velocity(:, 3), 'b');
        xlabel('Time');
        ylabel('Velocity');
        legend('X Velocity', 'Y Velocity', 'Z Velocity');
        title('Velocity vs Time');
        
        subplot(5,1,3);
        plot(timeSteps, acceleration(:, 1), 'r', timeSteps, acceleration(:, 2), 'g', timeSteps, acceleration(:, 3), 'b');
        xlabel('Time');
        ylabel('Acceleration');
        legend('X Acceleration', 'Y Acceleration', 'Z Acceleration');
        title('Acceleration vs Time');
        
        subplot(5,1,4);
        plot(timeSteps, quaternions(:, 1), 'r', timeSteps, quaternions(:, 2), 'g', timeSteps, quaternions(:, 3), 'b', timeSteps, quaternions(:, 4), 'k');
        xlabel('Time');
        ylabel('Quaternion');
        legend('q0', 'q1', 'q2', 'q3');
        title('Quaternion Orientation');
        
        subplot(5,1,5);
        plot3(position(:, 1), position(:, 2), position(:, 3), 'b');
        xlabel('X Position');
        ylabel('Y Position');
        zlabel('Z Position');
        title('3D Position Trajectory');
        grid on;
    else
        error('Hàm chỉ hỗ trợ vẽ đồ thị cho 1D, 2D hoặc 3D.');
    end
end
