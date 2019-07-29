function output=controlSystem(Sys,rho,eps,dt)

    Agents=Sys.Agents;
    ActivatedCenters=Sys.ActivatedCenters;
    m=size(Agents,1);
    output=Agents;
    
% for i=1:m
i=0;
while i<m
    i=i+1;
    
    curAgent=Agents(i);
    
    if ~curAgent.confliction.notCollided && curAgent.confliction.agent_num==0 % has a temporary collision center
        
        if  norm(curAgent.q_ci.coord-curAgent.confliction.leave(1:2))<1.5*rho && abs(curAgent.q_ci.dir-curAgent.confliction.leave(3))<pi/2  % ready to leave the center
            
            Agents(i)=curAgent;
            Sys.Agents=Agents;
            Sys=deactivateCenter(Sys,curAgent.confliction.center);
            Agents=Sys.Agents;
            curAgent=Agents(i);
            
            curAgent.speedmode=0;
            confliction.notCollided=1;
            curAgent.confliction=confliction;
            curAgent=controlFollowPath(curAgent,rho,eps,dt);
        else
            curAgent=controlAround(curAgent,curAgent.confliction.center,rho,eps,dt);
        end        
    else    % without temporary collision center, following the original control protocol
        Agents(i)=curAgent;
        Sys.Agents=Agents;        
        Sys=checkNoActivatedCenter(i,Sys);
        Agents=Sys.Agents;
        curAgent=Agents(i);     % update the Agent for checking temporary collision center
        
        if ~curAgent.confliction.notCollided && curAgent.confliction.agent_num==0 % if there is a new temporay collision center
            i=i-1; %this is tricky and dangerous
            continue;
        else    % if there is no new temporary collision center
            % following are original deconfliction protocol
            if ~curAgent.confliction.notCollided    %has a collision center
                
                if curAgent.confliction.isAround    %the collision is a around maneuver
                    
                    if norm(curAgent.q_ci.coord-curAgent.confliction.leave(1:2))<1.5*rho && abs(curAgent.q_ci.dir-curAgent.confliction.leave(3))<pi/2 % ready to leave the center
                        
                        % --- new, remove(deactivate) collision center
                        Agents(i)=curAgent; 
                        Sys.Agents=Agents;
                        Sys=deactivateCenter(Sys,curAgent.confliction.center);
                        Agents=Sys.Agents;
                        curAgent=Agents(i);
                        % ---                      
                        
                        
                        confliction.notCollided=1;
            %           curAgent.i=curAgent.confliction.leave_i;
                        curAgent.confliction=confliction;
                        curAgent=controlFollowPath(curAgent,rho,eps,dt);
                        Agents(i)=curAgent;
                                               
                        continue;

                    else    % not ready to leave the center
                        if  norm(curAgent.q_ci.coord-curAgent.confliction.center)<2*rho %approaching the center, do the round maneuver
                            
                            % --- new, add(activate) collision center
                            if ~curAgent.confliction.isActivated
                                Agents(i)=curAgent;
                                Sys.Agents=Agents;
                                Sys=activateCenter(Sys,curAgent.confliction.center);
                                disp('appraoching +1')
                                Agents=Sys.Agents;
                                curAgent=Agents(i);
                                
                                curAgent.confliction.isActivated=1;
                            end
                            
                            % ---
                            
                            curAgent.i=curAgent.confliction.leave_i;
                            curAgent.speedmode=0;
                            curAgent=controlAround(curAgent,curAgent.confliction.center,rho,eps,dt);                          
                            
                        else    % not approaching the center, just follow the path
%                             disp('!!!')
                            curAgent=controlFollowPath(curAgent,rho,eps,dt);
%                             Agents(i)=curAgent;
%                             Sys.Agents=Agents;
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
                Agents(i)=curAgent;
                Sys.Agents=Agents;
                Sys=checkNoSystemCollisionInit(i,Sys);  % check the collision
                Agents=Sys.Agents;
                curAgent=Agents(i);

                if curAgent.confliction.notCollided
                    curAgent=controlFollowPath(curAgent,rho,eps,dt);
                    Agents(i)=curAgent;
                    continue;
                else
                    i=i-1;
                    continue;
                end
            end
        end
    end
    Agents(i)=curAgent;
end

Agents(i)=curAgent;
Sys.Agents=Agents;
output=Sys;
end