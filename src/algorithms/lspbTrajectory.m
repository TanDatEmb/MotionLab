function [ q, qd, qdd, t, ppCell] = lspbTrajectory( wayPoints, numSamples, varargin)
    narginchk(2,6)

    % Convert strings to chars case by case for codegen support
    if nargin > 2
        charInputs = cell(1,nargin-2);
        [charInputs{:}] = convertStringsToChars(varargin{:});
    else
        charInputs = {};
    end

    % Check input validity
    validateattributes(wayPoints, {'numeric'}, {'2d','nonempty','real','finite'}, 'trapveltraj','wayPoints');
    validateattributes(numSamples, {'numeric'}, {'nonempty','scalar','real','finite'}, 'trapveltraj','m');

    % Establish some dimensions
    m = numSamples;
    n = size(wayPoints,1);
    p = size(wayPoints,2);

    % Parse inputs
    names = {'PeakVelocity', 'Acceleration', 'EndTime','AccelTime'};
    defaults = {[], [], [], []};
    parser = robotics.core.internal.NameValueParser(names, defaults);
    parse(parser, charInputs{:});
    vel = parameterValue(parser, names{1});
    acc = parameterValue(parser, names{2});
    tFi = parameterValue(parser, names{3});
    tAc = parameterValue(parser, names{4});

    % Input checks
    coder.internal.errorIf(p < 2, 'shared_robotics:robotcore:utils:WaypointsTooFew');
    coder.internal.errorIf(~isOptInputValid(n, p, vel, 'PeakVelocity'), 'shared_robotics:robotcore:utils:TrapVelOptInputSize', 'PeakVelocity');
    coder.internal.errorIf(~isOptInputValid(n, p, acc, 'Acceleration'), 'shared_robotics:robotcore:utils:TrapVelOptInputSize', 'Acceleration');
    coder.internal.errorIf(~isOptInputValid(n, p, tFi, 'EndTime'), 'shared_robotics:robotcore:utils:TrapVelOptInputSize', 'EndTime');
    coder.internal.errorIf(~isOptInputValid(n, p, tAc, 'AccelTime'), 'shared_robotics:robotcore:utils:TrapVelOptInputSize', 'AccelTime');

    % Ensure input values are matrices
    vel = reformatInput(vel, n, p);
    acc = reformatInput(acc, n, p);
    tFi = reformatInput(tFi, n, p);
    tAc = reformatInput(tAc, n, p);

    % Initialize outputs
    q = zeros(n, m);
    qd = zeros(n, m);
    qdd = zeros(n, m);

    % Compute parameters, coefficients, and breaks, and store for subsequent
    % use in pp-form and to generate outputs.
    parameterMat = zeros(n, p-1, 6);

    % Initialize the coefficient matrix to zeros. For P waypoints, there are
    % (P-1 segments) x (N dimensions) of polynomials, each with four
    % coefficients.
    coeffMat = zeros(3*(p-1)*n, 3);
    breakMat = zeros(n, 3*(p-1)+1);
    for i = 1:n
        for j = 1:p-1
            % Compute the trapezoidal profile parameters for the profile
            % segment between two waypoints. Since the user-specified
            % parameter values may be empty here, pass the indices as
            % inputs following function.
            [segVel, segAcc, segATime, segFTime] = computeProfileParams(i, j, wayPoints, vel, acc, tFi, tAc);
            parameterMat(i,j,:) = [wayPoints(i,j) wayPoints(i,j+1) ...
                                segVel segAcc segATime segFTime];
            [coefs, breaks] = computeScalarLSPBCoefficients(wayPoints(i,j), wayPoints(i,j+1), ...
                                                            segVel, segAcc, segATime, segFTime);

            % There are three elements that are computed by the scalar
            % coefficients, and they are entered as three rows in the
            % coefficient matrix
            coefIndex = false(3*(p-1)*n,1);
            lspbSegIndices = (3*(j-1)*n+i):n:(3*(j-1)*n+i+2*n);
            coefIndex(lspbSegIndices) = true;

            coeffMat(coefIndex, :) = coefs;
            breakMat(i, (3*j-2):(3*j+1)) = breaks + breakMat(i, (3*j-2));
        end
    end

    % Compute piecewise polynomial
    hasMultipleBreaks = checkPolyForMultipleBreaks(breakMat);
    [breaksCell, coeffsCell] = processPolynomialResults(breakMat, coeffMat, hasMultipleBreaks, p);

    % Create time vector and use it to evaluate piecewise polynomials
    t = linspace(0, max(sum(parameterMat(:,:,6),2)),m);

    % Define array sizes first so that cell may be defined in a single for
    % loop for codegen compatibility
    if hasMultipleBreaks
        numComputedPolynomials = n;
        indivPolyDim = 1;
    else
        numComputedPolynomials = 1;
        indivPolyDim = n;
    end

    ppCell = cell(numComputedPolynomials, 1);
    for jj = 1:numComputedPolynomials
        if hasMultipleBreaks
            rowSelection = jj;
            cellSelection = jj;
        else
            rowSelection = 1:n;
            cellSelection = 1;
        end
        [q(rowSelection,1:m), qd(rowSelection,1:m), qdd(rowSelection,1:m), ppCell{jj}] = ...
            generateTrajectoriesFromCoefs(breaksCell{cellSelection}, coeffsCell{cellSelection}, indivPolyDim, t);
    end
end

%% Helper Functions

function [q, qd, qdd, pp] = generateTrajectoriesFromCoefs(breaks, coeffs, dim, t)
%generateTrajectoriesFromCoefs Generate N-dimensional position, velocity, and acceleration trajectories
%   Given a set of breaks and coefficients, the dimension N, and a 1xM time
%   vector, T, this function computes the position vector, Q, as well as
%   its derivatives, QD and QDD, as M-dimensional row-vectors in time.

% Add flat segments to the start and end of each trajectory so the
% values will be held constant outside the user-specified time values
    [modBreaks, modCoeffs] = robotics.core.internal.addFlatSegmentsToPPFormParts(breaks, coeffs, dim);

    dCoeffs = robotics.core.internal.polyCoeffsDerivative(modCoeffs);
    ddCoeffs = robotics.core.internal.polyCoeffsDerivative(dCoeffs);

    pp = mkpp(modBreaks, modCoeffs, dim);
    ppd = mkpp(modBreaks, dCoeffs, dim);
    ppdd = mkpp(modBreaks, ddCoeffs, dim);

    coder.varsize('q', 'qd', 'qdd');

    q = ppval(pp, t);
    qd = ppval(ppd, t);
    qdd = ppval(ppdd, t);

end

function hasMultipleBreaks = checkPolyForMultipleBreaks(breakMat)
%hasMultipleBreaks Check if the polynomial has different breaks in different dimensions
%   In order for the piecewise polynomial to be represented as a single
%   pp-form, the output has to have one set of breaks that apply to all N
%   dimensions. This function checks the N row-vectors in the breakmat to
%   see whether they match or not (within a tolerance of eps). If they
%   match, the output flag hasMultipleBreaks is set to TRUE; otherwise, the
%   flag is set to FALSE.

    n = size(breakMat,1);

    % If the break sequences are all identical, then the vector may be
    % passed out as a single pp-form of dimension N
    hasMultipleBreaks = false;
    for i = 2:n
        hasMultipleBreaks = any(abs(breakMat(i-1,:)-breakMat(i,:)) > eps) || hasMultipleBreaks;
    end
end

function [breaksCell, coeffCell] = processPolynomialResults(breakMat, coeffMat, hasMultipleBreaks, p)
%processPolynomialResults Convert output to cell and add segments as needed
%   This utility performs a few key functions. First, it ensures that all
%   the dimensions of the polynomial share the same max break time, which
%   is achieved by adding flat segments from the end time to the max break
%   time. Second, it converts the uniformly sized breaks and coefficient
%   matrices into cell arrays. This is necessary as the addition of breaks
%   changes the number of segments in some dimensions of the output
%   piecewise polynomial.

    maxBreaksTime = max(breakMat(:,end));
    n = size(breakMat,1);

    % Output an Nx1 cell array either way
    breaksCell = cell(n,1);
    coeffCell = cell(n,1);

    % Assign elements to cell array
    for ii = 1:n
        if hasMultipleBreaks
            % Split out the polynomial coefficients and breaks
            nthCoeffIndex = false(n,1);
            nthCoeffIndex(ii) = true;
            coeffIndex = repmat(nthCoeffIndex, 3*(p-1), 1);
            coeffCell{ii} = coeffMat(coeffIndex,:);
            breaksCell{ii} = breakMat(ii,:);
        else
            coeffCell{ii} = coeffMat;
            breaksCell{ii} = breakMat(1,:);
        end
    end
end

function isValid = isOptInputValid(n, p, inputVar, inputName)
%isOptInputValid Returns true if the input is valid
%   The optional inputs are empty if unset (this is the default value), or
%   they may be scalar, Nx1 vectors, or Nx(P-1) matrices. Any other input
%   dimensions are invalid. This function returns true if the input is
%   empty, or if the input dimensions fit the valid ones and the input
%   passes other validation checks.

    isValid = true;
    if isempty(inputVar)
        return;
    end

    if ~(all(size(inputVar) == [1 1]) || all(size(inputVar) == [n 1]) || all(size(inputVar) == [n p-1]))
        isValid = false;
    else
        validateattributes(inputVar, {'numeric'}, {'real', 'positive', 'finite', 'nonnan'}, 'trapveltraj', inputName);
    end

end

function matrixInput = reformatInput(input, n, p)
%reformatInput Convert scalar and vector numeric inputs to Nx(P-1) matrix inputs
%   If values are provided as scalars or vectors, they need to be converted
%   to vectors so that they may be applied to each axis along each profile
%   segment. If the provided input is empty, the output will also be empty.

    matrixInput = input;
    if isscalar(input)
        matrixInput = repmat(input,n,p-1);
    elseif isvector(input)
        matrixInput = repmat(input,1,p-1);
    end
end

function [vParam, aParam, tAParam, tFParam] = computeProfileParams(i, j, wayPoints, Vel, Acc, TFi, TAc)
%computeProfileParams Compute parameters for a trapezoidal velocity profile
%   This function takes the user provided inputs and a set of waypoints and
%   outputs the corresponding completed set of parameters required for a
%   trapezoidal velocity profile: velocity, acceleration, acceleration
%   time, and final time. While all parameters are provided to this
%   function, they will be empty unless explicitly set by the user. If the
%   user inputs
%   cannot be satisfied, then an error is output that indicates the
%   constraint violation.
%
%   This function takes two indexing inputs I and J that specify the index
%   of the parameter: The parameters correspond to the Ith axis and Jth
%   profile segment (i.e. the profile connecting two waypoints) in that
%   axis. Additionally, J and J+1 specify the bounding indices of WAYPOINTS
%   of the profile segment for which the parameters are being computed.
%
%   The function outputs a complete set of the four scalar parameters that
%   may be used to generate a trapezoidal velocity profile for the profile
%   segment.

% Initialize outputs
    aParam = 0;
    vParam = 0;
    tAParam = 0;
    tFParam = 0;

    % Compute the initial and final profile segment position, scalars s0 and sF.
    s0 = wayPoints(i,j);
    sF = wayPoints(i,j+1);
    deltaSign = 1;

    %If (sF - s0) is negative, swap them while computing terms, and pass sign
    %to the output. This ensures that the upper and lower bound
    %inequalities can be computed without special case evaluation.
    if sF < s0
        [s0, sF] = swap(s0,sF);
        deltaSign = -1;
    end

    % Compute velocity, acceleration, acceleration time, and final time
    % based on provided inputs. To ensure a valid profile can be found, the
    % values for the product of velocity and final time, v*TF, must fall
    % between (sF-s0) and 2*(sF - s0). When a user specifies less than two
    % inputs, values are chosen so that v*tF is exactly in the middle of
    % the allowable bounds. Convert from binary to decimal for evaluation.
    % This is done manually to ensure codegen compatibility.
    inputCombo = [~isempty(Vel) ~isempty(Acc) ~isempty(TFi) ~isempty(TAc)];
    inputCombination = inputCombo*[2^3 2^2 2^1 2^0]';
    
    % Initialize variables to non-empty values for code generation
    % compatibility
    if inputCombo(1), VelSwitch = Vel; else, VelSwitch = ones(i,j); end
    if inputCombo(2), AccSwitch = Acc; else, AccSwitch = ones(i,j); end
    if inputCombo(3), TFiSwitch = TFi; else, TFiSwitch = ones(i,j); end
    if inputCombo(4), TAcSwitch = TAc; else, TAcSwitch = ones(i,j); end
    switch inputCombination
      case 12
        % Velocity and Acceleration
        vParam = VelSwitch(i,j);
        aParam = AccSwitch(i,j);

        % Solve for acceleration time and final time
        tAParam = vParam/aParam;
        tFParam = (vParam*tAParam + sF - s0)/vParam;
      case 10
        % Velocity and Final Time
        vParam = VelSwitch(i,j);
        tFParam = TFiSwitch(i,j);

        % Solve for acceleration time and acceleration
        tAParam = (s0 - sF + vParam*tFParam)/vParam;
        aParam = vParam/tAParam;
      case 9
        % Velocity and Acceleration Time
        vParam = VelSwitch(i,j);
        tAParam = TAcSwitch(i,j);

        % Solve for acceleration and final time
        aParam = vParam/tAParam;
        tFParam = (vParam*tAParam + sF - s0)/vParam;
      case 8
        % Velocity only
        % Choose the final time so that v*TF is in the middle of the
        % acceptable range.
        vParam = VelSwitch(i,j);
        tFParam = (1.5*(sF - s0)/vParam);

        % Solve for acceleration time and acceleration
        tAParam = (s0 - sF + vParam*tFParam)/vParam;
        aParam = vParam/tAParam;
      case 6
        % Acceleration and Final Time
        aParam = AccSwitch(i,j);
        tFParam = TFiSwitch(i,j);

        % Solve for acceleration time and velocity
        tACandidates = [((aParam*tFParam)/2 - (aParam*(aParam*tFParam^2 + 4*s0 - 4*sF))^(1/2)/2)/aParam, ...
                        ((aParam*tFParam)/2 + (aParam*(aParam*tFParam^2 + 4*s0 - 4*sF))^(1/2)/2)/aParam];

        % Ensure there are only positive, real roots to choose from
        tAParam = tACandidates((tACandidates > 0) & real(tACandidates));
        tAParam = tAParam(1); %Choose the smaller over the larger option if both remain
        vParam = (sF - s0)/(tFParam - tAParam);
      case 5
        % Acceleration and Acceleration Time
        aParam = AccSwitch(i,j);
        tAParam = TAcSwitch(i,j);

        % Solve for velocity and final time
        vParam = aParam*tAParam;
        tFParam = (vParam*tAParam + sF - s0)/vParam;
      case 4
        % Acceleration only.
        % Since the conditions apply bounds to the value of v*TF, choose
        % v*TF to be the average value, 1.5*(sF-s0), by solving for
        % acceleration time (tA) from the equations v*TF = sF-s0+v*TA and v
        % = a*tA.
        aParam = AccSwitch(i,j);
        tAParam = sqrt((sF - s0)/(2*aParam));

        % Solve for velocity and final time
        vParam = aParam*tAParam;
        tFParam = (sF - s0 + aParam*tAParam^2)/vParam;
      case 3
        % Final Time and Acceleration Time
        tFParam = TFiSwitch(i,j);
        tAParam = TAcSwitch(i,j);

        % Solve for velocity and acceleration
        vParam = (sF - s0)/(tFParam - tAParam);
        aParam = vParam/tAParam;
      case 2
        % Final Time only
        % Choose the velocity to be in the middle of the acceptable range.
        tFParam = TFiSwitch(i,j);
        vParam = 1.5*(sF - s0)/tFParam; %Choose V for the middle of the range

        % Solve for acceleration time and acceleration
        tAParam = (s0 - sF + vParam*tFParam)/vParam;
        aParam = vParam/tAParam;
      case 1
        % Acceleration Time only
        % Since the conditions apply bounds to the value of v*TF, choose
        % v*TF to be the average value, 1.5*(sF-s0), by solving for
        % acceleration (a) from the equations v*TF = sF - s0 + v*tA and
        % v = a*tA.
        tAParam = TAcSwitch(i,j);
        aParam = (sF - s0)/(2*tAParam^2);

        % Solve for velocity and final time
        vParam = aParam*tAParam;
        tFParam = (sF - s0 + aParam*tAParam^2)/vParam;
      case 0
        % None
        % Assume final time is 1 and choose velocity, v, to be in the
        % middle of the acceptable range.
        tFParam = 1;
        vParam = 1.5*(sF - s0)/tFParam;

        % Solve for acceleration time and acceleration
        tAParam = (s0 - sF + vParam*tFParam)/vParam;
        aParam = vParam/tAParam;
    end

    % Handle edge cases and verify that meaningful profile will be produced.
    % Generate an error if the minimum conditions are not met
    if s0 == sF
        % This will generate a flat profile. The only meaningful output
        % parameter is the final time. The acceleration time must be set to
        % be consistent with the general case.
        aParam = 0;
        vParam = 0;
        if isnan(tFParam) || (tFParam == 0)
            %Depending on the input, it may or may not be possible to compute
            %tF. This check ensures that a valid value is used.
            tFParam = 1;
        end
        tAParam = tFParam/3;
    else
        lowerBound = ((sF - s0)/tFParam < vParam);
        upperBound = (vParam <= 2*(sF - s0)/tFParam);
        coder.internal.errorIf(~lowerBound, 'shared_robotics:robotcore:utils:TrapVelLowerBoundCondition',i,j,j+1);
        coder.internal.errorIf(~upperBound, 'shared_robotics:robotcore:utils:TrapVelUpperBoundCondition',i,j,j+1);
    end

    % Apply the sign to the output
    vParam = deltaSign*vParam;
    aParam = deltaSign*aParam;

end

function [coefs, breaks] = computeScalarLSPBCoefficients(s0, sF, v, a, ta, tf)
%computeScalarLSPBCoefficients Compute linear segment with polynomial blends
%   Compute scalar LSPB profile given velocity, acceleration, acceleration
%   time, final time, and a time vector for the profile segment between two
%   waypoints. This consists of three polynomial sections that together
%   form a piecewise polynomial.

    breaks = [0 ta tf-ta tf];
    coefs = zeros(3,3);

    if v == 0
        % Flat profile
        coefs(:,3) = s0;
    else
        % Section 1: From t=0 to t=TAccel
        coefs(1,:) = [a/2, 0, s0];

        % Section 2: From t=TAccel to t=TFinal-TAccel
        coefs(2,:) = [0, v, a/2*ta^2 + s0];

        % Section 3: From t=TFinal-TAccel to t=TFinal
        coefs(3,:) = [-a/2, v, sF + a/2*ta^2 - v*ta];
    end

end

function [A,B] = swap(a,b)
%swap Swap two values
    A = b;
    B = a;
end
