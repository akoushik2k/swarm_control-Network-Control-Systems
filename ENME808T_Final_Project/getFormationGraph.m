function [A_formation] = getFormationGraph(numAgents,currentSubmission,currentWaypoint)
%GETFORMATIONGRAPH Produces a formation graph from the specification graph
% Use whatever criteria to select the edges you might want to enforce to
% meet the formation specification. A_formation should be a square
% adjacency matrix whose nonzero entries encode desired edge distances.
A_formation = zeros(numAgents);
switch currentSubmission
    % Define A_formation as desired
    case 'init'
        switch currentWaypoint
            case 1
                A_formation = [0 0.3 0 0.3 0.3 0.3;
                             0.3 0 0.3 0.3 0 0;
                             0 0.3 0 0.3 0 0;
                             0.3 0.3 0.3 0 0.3 0;
                             0.3 0 0 0.3 0 0.3;
                             0.3 0 0 0 0.3 0];
            case 2
                A_formation = [0 0.3 0 0.42 0.3 0.3;
                             0.3 0 0.3 0.3 0.42 0;
                             0 0.3 0 0.3 0 0;
                             0.42 0.3 0.3 0 0.3 0;
                             0.3 0.42 0 0.3 0 0.3;
                             0.3 0 0 0 0.3 0];
            case 3
                A_formation = 1.2*[0 0.212 0.3 0 0 0.212;
                             0.212 0 0.212 0 0 0.3;
                             0.3 0.212 0 0.212 0.3 0.212;
                             0 0 0.212 0 0.212 0.3;
                             0 0 0.3 0.212 0 0.212;
                             0.212 0.3 0.212 0.3 0.212 0];             
        end
    case 'room'
        % Define A_formation as desired
        A_formation = 10*[0 0.212 0.3 0 0 0.212;
                             0.212 0 0.212 0 0 0.3;
                             0.3 0.212 0 0.212 0.3 0.212;
                             0 0 0.212 0 0.212 0.3;
                             0 0 0.3 0.212 0 0.212;
                             0.212 0.3 0.212 0.3 0.212 0];
    case 'return'
        switch currentWaypoint
            case 4
                A_formation = 1.1*[0 0.3 0 0.42 0.3 0.3;
                             0.3 0 0.3 0.3 0.42 0;
                             0 0.3 0 0.3 0 0;
                             0.42 0.3 0.3 0 0.3 0;
                             0.3 0.42 0 0.3 0 0.3;
                             0.3 0 0 0 0.3 0];
            case 5
                A_formation = [0 0.3 0.6 0 0 0;
                             0.3 0 0.3 0.6 0 0;
                             0.6 0.3 0 0.3 0.6 0;
                             0 0.6 0.3 0 0.3 0.6;
                             0 0 0.6 0.3 0 0.3;
                             0 0 0 0.6 0.3 0];
            case 6
                A_formation = [0 0.3 0.6 0 0 0;
                             0.3 0 0.3 0.6 0 0;
                             0.6 0.3 0 0.3 0.6 0;
                             0 0.6 0.3 0 0.3 0.6;
                             0 0 0.6 0.3 0 0.3;
                             0 0 0 0.6 0.3 0];
            case 7
                A_formation = [0 0.3 0.6 0 0 0;
                             0.3 0 0.3 0.6 0 0;
                             0.6 0.3 0 0.3 0.6 0;
                             0 0.6 0.3 0 0.3 0.6;
                             0 0 0.6 0.3 0 0.3;
                             0 0 0 0.6 0.3 0];
                
            case 8
                A_formation = 1*[0 0.3 0.3 0 0 0;
                             0.424 0 0.3 0.424 0.3 0;
                             0.424 0.3 0 0.3 0.424 0;
                             0 0.424 0.3 0 0.3 0.424;
                             0 0.3 0.424 0.3 0 0.424;
                             0 0 0 0.424 0.424 0];

                
        end
end
