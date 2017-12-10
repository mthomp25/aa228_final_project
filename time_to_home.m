% find fastest way home for each sample

% returns a vector of times, in units of timesteps
% the index of the time vector corresponds to the sample number
function time = time_to_home(env)

num_samples = length(env.samples(1,:)); % number of samples
num_tsteps = env.missionLength/env.ts; % Number of time steps
home = env.home;

policy = value_iter(env);

p = cell(1,num_samples);
time = zeros(1, num_samples);

for i = 1:num_samples
    t = 0;
    p_seq = zeros(num_tsteps,1);
    pos = env.samples(1, i);
    
    for k = 1:length(p_seq)
        p_seq(k) = policy(pos);
        
        [dt, pos] = moveOneSquare(env, pos, p_seq(k));
        t = t + dt;
        
        if pos == home
            p{i} = p_seq(1:k);
            time(i) = t;
            disp(['sample ' num2str(i) ' time to home: ' num2str(t)])
            break
        end
        
    end
end

end

% value iteration function (just iterates over the gridworld)
function policy = value_iter(env)

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
            [r(a), posnew] = reward(env, k, a);
            Usa(a) = U(posnew);
        end
        
        [U(k), policy(k)] = max(r+gamma*Usa);
    end
end


end

% reward function, penalize time, home reward is 10
function [r, posnew] = reward(env, pos, a)

    [dt, posnew] = moveOneSquare(env, pos, a);
    
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

% move one square is nearly identical to the main value iteration function
function [dt, posnew] = moveOneSquare(env, pos, a)
    [row,col] = ind2sub([env.rows, env.rows], pos);
    dt = 0;
    posnew = pos;
    
    switch a
        case 1
            % move up
            if (row~=1) % don't go off the map
                m = (env.topo(row-1,col) - env.topo(row,col))/env.d; % slope of this move
                m = max(0, m);
                dt = m+1;  % see definition of speed
                posnew = sub2ind([env.rows, env.rows], row-1, col);
            end
        
        case 2
            % move right
            if (col~=env.rows) % don't go off the map
                m = (env.topo(row,col+1) - env.topo(row,col))/env.d; % slope of this move
                m = max(0, m);
                dt = m+1;  % see definition of speed
                posnew = sub2ind([env.rows, env.rows], row, col+1);
            end
        
        case 3
            % move down
            if (row~=env.rows) % don't go off the map
                m = (env.topo(row+1,col) - env.topo(row,col))/env.d; % slope of this move
                m = max(0, m);
                dt = m+1;  % see definition of speed
                posnew = sub2ind([env.rows, env.rows], row+1, col);
            end

        otherwise
            % move left
            if (col~=1) % don't go off the map
                m = (env.topo(row,col-1) - env.topo(row,col))/env.d;
                m = max(0, m);
                dt = m+1;  % see definition of speed
                posnew = sub2ind([env.rows, env.rows], row, col-1);
            end
    end
end



