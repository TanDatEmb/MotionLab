function [endTimes,peakVels] = helperProfileForMaxVel(wpts,maxVelocity)
    %helperProfileForMaxVel Generate parameters for a trapezoidal velocity profile so it will meet velocity constraints
    %   This function accepts the waypoints and velocity limits, and from
    %   that computes a set segment end times and peak velocities that
    %   produces a fast trapezoidal profile that treats the velocity limit
    %   as an upper bound.

    %   Copyright 2021 The MathWorks, Inc.
    
    % Make sure the velocity limits are given as an Nx1 vector
    validateattributes(maxVelocity,{'numeric'},{'scalar'});

    % The waypoints matrix is composed of P waypoints, each of dimension N.
    % Find the Nx(P-1) matrix of segment lengths along each dimension.
    segLengths = abs(diff(wpts,1,2));

    % If all dimensions obey the same velocity limits, the end time will be
    % determined from the dimension longest segment. Set the peak velocity
    % for that segment to the max velocity and use that to find the
    % matching minimum endTime for each segment from the known constraints:
    %    1) peakVelocity > (s(endTime) - s(0))/endTime --> endTime > segLength/peakVelocity
    %    2) peakVelocity <= 2*(s(endTime) - s(0)/endTime) --> endTime <= 2*segLength/peakVelocity
    maxSegLengths = max(segLengths,[],1);
    endTimeLowerBound = (maxSegLengths/maxVelocity);

    % The end time has to be greater than the lower bound, so multiply by a
    % factor greater than 1. This has the effect of choosing the
    % acceleration.
    greaterThanFactor = 1.1;
    endTimes = repmat(greaterThanFactor*endTimeLowerBound,size(wpts,1),1);

    % Given this end time, determine the min and max peak velocity for each
    % segment and dimension, then choose the largest value that doesn't
    % exceed the maximum velocity
    minPeakVels = segLengths./endTimes;
    maxPeakVels = 2*segLengths./endTimes;

    peakVels = min(maxPeakVels,maxVelocity);
    peakVels = max(minPeakVels,peakVels);

    % There cannot be zero-values for peak velocity
    peakVels(peakVels==0) = maxVelocity;
end