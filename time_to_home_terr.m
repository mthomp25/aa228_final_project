% find fastest way home for each sample

% input: terr - a vector of length max(env.zones) which gives the terrain
% value (0-3) of the zone corresponding to the index of terr.
% terr(zone_num) = -1 is terrain in that zone is unknown

% returns a 2-d vector of times, in units of timesteps
% first column is the most optimistic time home, second is the most
% pessimistic

% the row index of the time vector corresponds to the sample number
function time = time_to_home_terr(env, terr)

num_samples = length(env.samples(1,:)); % number of samples
num_tsteps = env.missionLength/env.ts; % Number of time steps
home = env.home;

% terr_good sets each unknown terrain zone to 0
terr_good = terr;
terr_good(terr<0) = 0;
% terr_bad sets unkown terrain to 3
terr_bad = terr;
terr_bad(terr<0) = 3;

% run value iteration for both good and bad
policy_good = value_iter(env, terr_good);
policy_bad = value_iter(env, terr_bad);

p_g = cell(num_samples, 1); %record p_seq to get home
p_b = cell(num_samples, 1);

time = zeros(num_samples, 2); %initialize time

for i = 1:num_samples
    t = 0;
    p_seq = zeros(num_tsteps,1);
    pos = env.samples(1, i);
    
    for k = 1:length(p_seq) %iterate for good policy
        p_seq(k) = policy_good(pos);
        
        [dt, pos] = moveOneSquare(env, terr_good, pos, p_seq(k));
        t = t + dt;
        
        if pos == home
            p_g{i} = p_seq(1:k);
            time(i, 1) = t;
            break
        end
        
    end
    
    % do exact same thing for bad policy
    t = 0;
    p_seq = zeros(num_tsteps,1);
    pos = env.samples(1, i);
    for k = 1:length(p_seq) %iterate for bad policy
        p_seq(k) = policy_bad(pos);
        
        [dt, pos] = moveOneSquare(env, terr_bad, pos, p_seq(k));
        t = t + dt;
        
        if pos == home
            p_b{i} = p_seq(1:k);
            time(i,2) = t;
            break
        end
        
    end
    disp(['sample ' num2str(i) ' time to home: ' num2str(time(i,:))])
end

end

% value iteration function (just iterates over the gridworld)
function policy = value_iter(env, terr)

num_actions = 4;
gridSize = env.rows*env.rows; %number of possible positions
gamma = .9;

U = zeros(gridSize, 1);
policy = zeros(size(U));

for i = 1:100
    for k = 1:length(U);
        
        Usa = zeros(num_actions,1);
        r = zeros(size(Usa));
        
        for a = 1:num_actions
            [r(a), posnew] = reward(env, terr, k, a);
            Usa(a) = U(posnew);
        end
        
        [U(k), policy(k)] = max(r+gamma*Usa);
    end
end


end

% reward function, penalize time, home reward is 10
function [r, posnew] = reward(env, terr, pos, a)

    [dt, posnew] = moveOneSquare(env, terr, pos, a);
    
    if pos == env.home
        r = 10;
    else
        if dt > 0
            r = -dt;
        else %invalid action
            r = -10;
        end
    end

end

% move one square is nearly identical to the main value iteration with terrain function
function [dt, posnew] = moveOneSquare(env, terr, pos, a)
    [row,col] = ind2sub([env.rows, env.rows], pos);
    rownew = row;
    colnew = col;
    m = 0;
    dt = 0;
    
    switch a
        case 1
            % move up
            if (row~=1) % don't go off the map
                m = (env.topo(row-1,col) - env.topo(row,col))/env.d; % slope of this move
                m = max(0, m);
                rownew = row-1;
            end
        
        case 2
            % move right
            if (col~=env.rows) % don't go off the map
                m = (env.topo(row,col+1) - env.topo(row,col))/env.d; % slope of this move
                m = max(0, m);
                colnew = col+1;
            end
        
        case 3
            % move down
            if (row~=env.rows) % don't go off the map
                m = (env.topo(row+1,col) - env.topo(row,col))/env.d; % slope of this move
                m = max(0, m);
                rownew = row+1;
            end

        otherwise
            % move left
            if (col~=1) % don't go off the map
                m = (env.topo(row,col-1) - env.topo(row,col))/env.d;
                m = max(0, m);
                colnew = col-1;
            end
    end
    
    % NOTE: THIS HAS BEEN CHANGED.
    % posnew = pos if it hits the edge, which should be enough
    % we used to have dt = 0 also
    posnew = sub2ind([env.rows, env.rows], rownew, colnew);
    
    if pos ~= posnew
        dt = terr(env.zones(rownew,colnew))+m+1;  % see definition of speed
    end
end
