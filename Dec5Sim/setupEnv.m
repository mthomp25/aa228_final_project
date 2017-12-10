function env = setupEnv(level)
% Set up the environment structure based on the given complexity level.
% INPUT
%   level - the desired complexity level
%       0 - 7x7 gridworld, flat, smooth, samples all have the same value
%       1 - 7x7 gridworld, valley, smooth, samples all have the same value
%       2 - 7x7 gridworld, valley, 4 terrain zones, samples all have the 
%           same value
%       3 - 7x7 gridworld, valley, 4 terrain zones, samples have different
%           values
% OUTPUT
%   env - structure describing the environment, the rover knows everything
%       given in this structure. Fields:
%       width - width of the entire map [m]
%       rows - number of rows/columns in the grid
%       d - width of a single gridspace
%       home - index location of the home base (rocket launch pad)
%       vb - base speed of rover [m/h]
%           *if the rover is traveling at vb it will take 2*ts to get
%            through the gridspace.
%       ts - time step [h]
%       missionLength - time from start until rocket launches [h]
%       samples - 2x(#samples) matrix, [location index; sample value]
%       topo - rowsxrows matrix with the elevation of the center of each
%           respective gridspace.
%       zones - rowsxrows matrix with the respective zone number of each
%           respective gridspace. Each terrain zone has a unique zone 
%           number.
%--------------------------------------------------------------------------
% Kaitlin Dennison - Stanford University - AA228 Aut 2017
% Last Updated 12/5/2017
%--------------------------------------------------------------------------

%% variables
% v - current speed of rover
% vb - base speed of rover
% g - gridspace [row col]
% a - action: 1=up, 2=right, 3=down, 4=left
% w - weather severity, [0,3], 0 = clear
% r - terrain roughness, [0,3], 0 = smooth
% rz - terrain zone
% h - height/elevation at center of gridspace
% m - slope at center of gridspace, [-1,1]
% s - sample
% t - time
% ti - time in
% to - time out
% ts - time step
% nts - number of times steps
% d - side length of a gridspace

%% environment structure
% grid
    env.width = 7000; %[m]
    env.rows = 7;
    env.d = env.width/env.rows; %[m]
    if mod(env.rows,2) == 0
        % even
        env.home = sub2ind([env.rows, env.rows], (env.rows)/2, (env.rows)/2);
            % up and left of center
    else
        %odd
        env.home = sub2ind([env.rows, env.rows], (env.rows+1)/2, (env.rows+1)/2);
    end
% speed/time
    % need to make sure dt is an integer...
    % nts = (m+2)+r+w
    % nts = 2 if perfect conditions
    % v = d/(nts*ts)
    env.vb = 3000/24; %[m/h]
    env.ts = env.d/(2*env.vb); %[h]
    env.missionLength = 25*env.ts; %2*env.width/env.vb; %[h]
% samples
    env.samples = [6 9 18 37 41 28 46;50.*ones(1,7)]; %[g;val]
% topography
    env.topo = zeros(env.rows);
% terrain
    env.zones = ones(env.rows);        

switch level
    case 1 % add in topography
        % topography
            env.topo = [2 2 2 2 2 2 2; ...
                        2 2 1 1 1 1 1; ...
                        1 1 1 0 0 0 1; ...
                        1 1 0 0 0 0 1; ...
                        0 1 1 1 1 1 1; ...
                        0 1 1 1 1 1 2; ...
                        1 1 1 1 2 2 2].*env.d;
        % Katie's best path:
        %    a = [4 4 1 1 2 2 2 2 2 3 3 3 4 3 4 4 3 1 1 1 0]
        %    #samples: 6/7
        %    score: 300
        %    time: 88
    case 2
        % topography
            env.topo = [0 0 0 1 2 2 2; ...
                        0 0 0 1 2 3 2; ...
                        0 0 0 1 2 2 2; ...
                        0 0 1 1 1 1 1; ...
                        1 1 1 1 1 0 0; ...
                        1 1 1 1 1 1 1; ...
                        1 1 1 1 1 1 1].*env.d;
            % this works fine as a matrix for small grids but if we want a large
            % gridworld then topo should be a function of position not a matrix of
            % values.
        % Katie's best path:
        %    a = [4 1 1 4 4 3 3 3 3 3 2 2 2 2 2 1 1 1 2 4 4 4 0]
        %    #samples: 6/7
        %    score: 300
        %    time: 96
    case 3 % add in terrain
        % samples
            env.missionLength = 30*env.ts;
            env.samples = [6 9 18 37 41;50.*ones(1,5)]; %[g;val]
        % topography
            env.topo = [2 2 2 2 2 2 2; ...
                        2 2 1 1 1 1 1; ...
                        1 1 1 0 0 0 1; ...
                        1 1 0 0 0 0 1; ...
                        0 1 1 1 1 1 1; ...
                        0 1 1 1 1 1 2; ...
                        1 1 1 1 2 2 2].*env.d;
        % terrain
            env.zones = [2 2 2 3 3 2 2; ...
                         2 2 1 3 3 4 4; ...
                         1 1 1 3 2 4 4; ...
                         3 3 3 4 2 2 1; ...
                         3 3 4 4 4 1 1; ...
                         1 1 1 4 3 3 1; ...
                         1 2 2 2 3 3 3];
        % Katie's best path:
        %    a = [4,1,1,4,3,4,3,3,3,2,2,1,1,2,0]
        %    #samples: 3/5
        %    score: 150
        %    time: 112
    case 4 % Mountain with terrain
        % samples
            env.missionLength = 30*env.ts;
            env.samples = [6 9 18 37 41;50.*ones(1,5)]; %[g;val]
        % topography
            env.topo = [0 0 0 1 2 2 2; ...
                        0 0 0 1 2 3 2; ...
                        0 0 0 1 2 2 2; ...
                        0 0 1 1 1 1 1; ...
                        1 1 1 1 1 0 0; ...
                        1 1 1 1 1 1 1; ...
                        1 1 1 1 1 1 1].*env.d;
        % terrain
            env.zones = [2 2 3 4 4 4 1; ...
                         2 3 3 4 3 3 1; ...
                         2 4 1 4 3 1 1; ...
                         4 4 1 1 3 1 2; ...
                         3 2 2 1 4 2 2; ...
                         3 2 4 4 4 4 3; ...
                         3 1 1 1 3 3 3];
        % Katie's best path:
        %    a = [1,1,4,2,2,2,2,3,3,4,4,0]
        %    #samples: 3/5
        %    score: 150
        %    time: 92
    case 5 % add in terrain
        % mission length
            env.missionLength = 40*env.ts;
        % topography
            env.topo = [2 2 2 2 2 2 2; ...
                        2 2 1 1 1 1 1; ...
                        1 1 1 0 0 0 1; ...
                        1 1 0 0 0 0 1; ...
                        0 1 1 1 1 1 1; ...
                        0 1 1 1 1 1 2; ...
                        1 1 1 1 2 2 2].*env.d;
        % terrain
            env.zones = [2 2 2 3 3 2 2; ...
                         2 2 1 3 3 4 4; ...
                         1 1 1 3 2 4 4; ...
                         3 3 3 4 2 2 1; ...
                         3 3 4 4 4 1 1; ...
                         1 1 1 4 3 3 1; ...
                         1 2 2 2 3 3 3];
        % Katie's best path:
        %    a = [4 4 3 3 4 2 2 2 2 2 1 2 1 1 1 4 4 4 3 3 0]
        %    #samples: 5/7
        %    score: 250
        %    time: 160
    case 6 % Mountain with terrain
        % mission length
            env.missionLength = 40*env.ts;
        % topography
            env.topo = [0 0 0 1 2 2 2; ...
                        0 0 0 1 2 3 2; ...
                        0 0 0 1 2 2 2; ...
                        0 0 1 1 1 1 1; ...
                        1 1 1 1 1 0 0; ...
                        1 1 1 1 1 1 1; ...
                        1 1 1 1 1 1 1].*env.d;
        % terrain
            env.zones = [2 2 3 4 4 4 1; ...
                         2 3 3 4 3 3 1; ...
                         2 4 1 4 3 1 1; ...
                         4 4 1 1 3 1 2; ...
                         3 2 2 1 4 2 2; ...
                         3 2 4 4 4 4 3; ...
                         3 1 1 1 3 3 3];
        % Katie's best path:
        %    a = [4 1 1 4 2 2 2 2 2 3 3 4 4 4 3 3 3 1 1 1 0]
        %    #samples: 5/7
        %    score: 250
        %    time: 152
    otherwise
        % leave as base case - level 0
end
