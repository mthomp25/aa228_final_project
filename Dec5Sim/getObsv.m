function [tf, g]  = getObsv(ti,gi,a,env)
% Observe the time it takes to move between gridspaces
% INPUT
%   ti - initial time (time left the initial gridspace) [h]
%   gi - the initial gridspace index (NOT subscripts)
%   a - action to move to the new gridspace (1-up, 2-right, 3-down, 4-left)
%       *deterministic
%   env - the environment structure for the current simulation 
%       *see setupEnv
% OUTPUT
%   tf - final time (time to reach the center of the new gridspace) [h]
%   g - new gridspace index
% NOTES
%   - getObsv assumes all movement is in the terrain zone of the new 
%     gridspace even though it is really half in the initial and half in 
%     the final gridspace terrain zone.
%   - If the rover tries to move off of the map getObsv will just return ti
%     and gi.
%--------------------------------------------------------------------------
% Kaitlin Dennison - Stanford University - AA228 Aut 2017
% Last Updated 12/5/2017
%--------------------------------------------------------------------------

%% Check to make sure the secret environment has been established
persistent secret;
    if isempty(secret)
        % Allocate secret data from the environment
        % secret will only exist inside getObsv so no user can change the
        % data inside the structure outside of this function. It will
        % remain in memory until the workspace is cleared. It only needs to
        % be allocated once.
        secret = setupSecret(env);
    end

%% Move to the new gridspace
[gr, gc] = ind2sub([env.rows env.rows],gi); % get row and col subscripts of gi

switch a
    case 1
        % up
        grn = gr-1; % new row subscript
        gcn = gc; % new col subscript
    case 2
        % right
        grn =  gr;
        gcn = gc+1;
    case 3
        % down
        grn =  gr+1;
        gcn = gc;
    otherwise
        % left
        grn = gr;
        gcn = gc-1;
end

if grn < 1 || gcn < 1 || grn > env.rows || gcn > env.rows
    % attempted to go off the map, do not move
    g = gi;
    tf = ti;
    return
else
    g = sub2ind([env.rows env.rows], grn, gcn);
end

%% Get the final time
m = (env.topo(g)-env.topo(gi))/env.d; % slope between centers of gridspaces
if m < 0
    m = 0; % rover moves at same speed as flat when going downhill
end
rz = env.zones(g); % terrain zone of the final gridspace
nts = (m+1)+secret.r(rz); %+secret.w(g,ti); % number of time steps to move
tf = ti + nts*env.ts; 

end
