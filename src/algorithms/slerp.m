function q = slerp(~, q1, q2, t)
% Calculate the angle between them
dotProduct = dot(q1, q2);
if dotProduct < 0.0
    q1 = -q1;
    dotProduct = -dotProduct;
end

DOT_THRESHOLD = 0.9995;
if dotProduct > DOT_THRESHOLD
    % If the inputs are too close for comfort, linearly interpolate and normalize the result.
    q = q1 + t*(q2 - q1);
    q = q / norm(q);
    return;
end

% Acos is safe to use because dotProduct is clamped in [0, DOT_THRESHOLD]
theta_0 = acos(dotProduct);  % theta_0 = angle between input vectors
theta = theta_0*t;  % theta = angle between q1 and result

sin_theta = sin(theta);  % compute this value only once
sin_theta_0 = sin(theta_0);  % compute this value only once

s0 = cos(theta) - dotProduct * sin_theta / sin_theta_0;  % scale factor for q1
s1 = sin_theta / sin_theta_0;  % scale factor for q2

q = (s0 * q1) + (s1 * q2);
end