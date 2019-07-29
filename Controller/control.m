function output=control(Agents,rho,eps,dt)

m=size(Agents,1);
output=Agents;
for i=1:m
    curAgent=Agents(i);
    if ~curAgent.confliction.notCollided    %has a collision center
        if curAgent.confliction.isAround    %the collision is a around maneuver
            if norm(curAgent.q_ci.coord-curAgent.confliction.leave(1:2))<1.5*rho && abs(curAgent.q_ci.dir-curAgent.confliction.leave(3))<pi/2 % ready to leave the center
                confliction.notCollided=1;
%                 curAgent.i=curAgent.confliction.leave_i;
                curAgent.confliction=confliction;
                Agents(i)=curAgent;
                continue;
                
            else    % not ready to leave the center
                if norm(curAgent.q_ci.coord-curAgent.confliction.center)<2*rho %approaching the center, do the round maneuver
                    curAgent.i=curAgent.confliction.leave_i;
                    curAgent.speedmode=0;
                    curAgent=controlAround(curAgent,curAgent.confliction.center,rho,eps,dt);
                else    % not approaching the center, just follow the path
                    curAgent=controlFollowPath(curAgent,rho,eps,dt);
                end
            end
        else    % the collision is a chasing maneuver
            if norm(curAgent.path(curAgent.i+1,1:2)-curAgent.confliction.center) > norm(curAgent.path(curAgent.i-1,1:2)-curAgent.confliction.center)    % leaving the center
                confliction.notCollided=1;
                curAgent.speedmode=0;
                curAgent.confliction=confliction;
            % else    % not leaving
            end
            curAgent=controlFollowPath(curAgent,rho,eps,dt);          
        end
       
    else    % no current collision center
        for j=1:m   % collision check with all the agents except himself
            if j~=i
                [confliction1, confliction2]=checkNoAgentCollisionInit(curAgent,Agents(j));
                if ~confliction1.notCollided % find the collision
                    curAgent.confliction=confliction1;
                    curAgent.speedmode=confliction1.speedmode;
%                     curAgent.i=curAgent.confliction.leave_i;
                    Agents(i)=curAgent;
                    
%                     Agents(j).i=confliction2.leave_i;
                    Agents(j).confliction=confliction2;
                    Agents(j).speedmode=confliction2.speedmode;
                    continue;
                end
            end
        end
        
        if curAgent.confliction.notCollided
            curAgent=controlFollowPath(curAgent,rho,eps,dt);
            Agents(i)=curAgent;
            continue;
        end   
    end
    Agents(i)=curAgent;
end

output=Agents;
end