function cpts = helperCreateControlPointsFromWaypoints(wpts)

L = length(wpts) - 1;

% Form matrices used to compute interior points of control polygon
r = zeros(L+1, size(wpts,1));
A = eye(L+1);
for i= 1:(L-1)
    A(i+1,(i):(i+2)) = [1 4 1];
    r(i+1,:) = 6*wpts(:,i+1)';
end

% Override end points
A(2,1:3) = [3/2 7/2 1]; 
A(L,(L-1):(L+1)) = [1 7/2 3/2]; 

% choose r0 and rL
r(1,:) = (wpts(:,1) + (wpts(:,2) - wpts(:,1))/2)';
r(end,:) = (wpts(:,end-1) + (wpts(:,end) - wpts(:,end-1))/2)';

dInterior = (A\r)';

% Construct complete control polygon
cpts = [wpts(:,1) dInterior wpts(:,end)];

end