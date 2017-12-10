function reward = getRew(t,g,env)
% Get the reward for being on a certain gridspace 
% INPUT
%   t - current time
%   g - index of the current gridspace
%   env - the environment structure for the current simulation 
%       *see setupEnv
% OUTPUT
%   reward - reward earned for simply being in that square. 
%       - If there is a sample in g, it will be picked up but not redeemed. 
%       - If the rover is in the home gridspace all non-redeemed, picked-up
%         samples will be redeemed.
%       - If the rover is in a terrain zone that it has not visited yet, it
%         will immediately receive the reward for being in a new zone.
%--------------------------------------------------------------------------
% Kaitlin Dennison - Stanford University - AA228 Aut 2017
% Last Updated 12/5/2017
%--------------------------------------------------------------------------

%% Initialize

reward = 0;

%% Variables to keep track of samples picked up and zones visited
% These variables exist only inside this function and will be initialized
% the first time getRew is called. They will persist in the workspace until
% the workspace is cleared. (They do not disappear when the function has
% completed its call.)
persistent pickedUp; % booleans for which samples have been picked up
persistent redeemed; % booleans for which samples have been returned to the rocket to redeem for a reward
if isempty(pickedUp)
    % initialize
    pickedUp = zeros(1,size(env.samples,2));
    redeemed = zeros(1,size(env.samples,2));
end
% persistent visited; % booleans for which terrain zones have been visited
% if isempty(visited)
%     % initialize
%     visited = zeros(1,max(max(env.zones)));
% end

%% Pick up or redeem samples

s = find(env.samples(1,:) == g); % indices of samples in g
if ~isempty(s)
    pickedUp(s) = 1;
end

if g == env.home && t <= env.missionLength
    f = find(pickedUp == 1); % indices of picked up samples
    if ~isempty(f)
        for j = f
            if redeemed(j) == 0
                % only give the reward if the sample has not yet been
                % redeemed
                reward = reward + sum(env.samples(2,j));
                redeemed(j) = 1;
            end
        end
    end
end

%% Give a reward for visiting a new terrain zone if necessary.

% rz = env.zones(g);
% if visited(rz) == 0
%     reward = reward + 10;
% end
% visited(rz) = 1;

end
