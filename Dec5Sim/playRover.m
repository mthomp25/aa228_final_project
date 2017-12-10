function [score, time, path] = playRover(a,env)
% play a single or a sequence of actions
% INPUT
%   a - action to move to the new gridspace (1-up, 2-right, 3-down, 4-left)
%       *deterministic
%       *enter 0 to end/reset the game
%       *can be a vector of actions representing a sequence
%   env - the environment structure for the current simulation 
%       *see setupEnv
% OUTPUT
%   score - the current accumumated score
%   time - the time after the action was executed
% NOTES
%   - playRover will keep track of the time, location, and the accumulated
%     reward of the current game. 
%   - a map with the final location of the rover will be displayed.
%--------------------------------------------------------------------------
% Kaitlin Dennison - Stanford University - AA228 Aut 2017
% Last Updated 12/5/2017
%--------------------------------------------------------------------------

%% Initialize
persistent points
    if isempty(points)
        points = 0;
    end
persistent t
    if isempty(t)
        t = 0;
    end
persistent g
    if isempty(g)
        g = env.home;
    end
persistent p
    if isempty(p)
        p = g;
    end
persistent as
    if isempty(as)
        as = a;
    end
    
%% End game?
if a == 0
    disp(['Final score: ', num2str(points), ', Final time: ', num2str(t)])
    score = points;
    time = t;
    path = as;
    resetGame()
    return
end

%% Play

if a == 5
    in = input('Action: ');
    as = in;
    while in ~= 0
        [tf, g]  = getObsv(t,g,in,env);
        if tf == t
            disp('Invalid action - attempted to move off the map')
        end
        t = tf;
        if t >= env.missionLength
            disp('Mission time limit reached')
        end
        points = points + getRew(t,g,env);
        p = [p,g];
        displayMap(p,env)
        score = points;
        time = t;
        path = as;
        disp(['Score: ', num2str(points), ', Time: ', num2str(t)])
        in = input('Action: ');
        as = [as,in];
    end
    disp(['Final score: ', num2str(points), ', Final time: ', num2str(t)])
    resetGame()
    return
else
    for i = 1:length(a)
        if a(i) == 0
            disp(['Final score: ', num2str(points), ', Final time: ', num2str(t)])
            score = points;
            time = t;
            path = as;
            displayMap(p,env)
            resetGame()
            return
        end
        [tf, g]  = getObsv(t,g,a(i),env);
        if tf == t
            disp('Invalid action - attempted to move off the map')
        end
        t = tf;
        if t >= env.missionLength
            disp('Mission time limit reached')
        end
        points = points + getRew(t,g,env);
        p = [p,g];
    end
    displayMap(p,env)
    score = points;
    time = t;
end

end