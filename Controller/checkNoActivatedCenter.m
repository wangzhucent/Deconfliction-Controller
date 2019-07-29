function outputSys=checkNoActivatedCenter(i,Sys)
rho=40;

ActivatedCenters=Sys.ActivatedCenters;
Agents=Sys.Agents;

for j=1:size(ActivatedCenters,1)
    if  (Agents(i).confliction.notCollided || ( ~Agents(i).confliction.isActivated && norm(ActivatedCenters(1:2)-Agents(i).confliction.center) > 5 ) ) && norm(ActivatedCenters(1:2)-Agents(i).q_ci.coord)<1.5*rho
        
        % cancel the deconfliction of this Agents with others; The other
        % agent won't be in process of deconfliction, or have a temporary
        % confliction center

        if ~Agents(i).confliction.notCollided
            confliction.notCollided=1;
            Agents(Agents(i).confliction.agent_num).confliction=confliction;
        end
        
        % update the current agent confliction state
        
        output.notCollided=0;
        output.agent_num=0;
        % no attribute "approach"
        leave_i1=Agents(i).i;
        leave_l1=norm(Agents(i).path(leave_i1,1:2)-Agents(i).q_ci.coord);
        while norm(Agents(i).path(leave_i1,1:2)-ActivatedCenters(j,1:2))<2*rho && leave_i1<size(Agents(i).path,1)
            leave_l1=leave_l1+norm(Agents(i).path(leave_i1+1,1:2)-Agents(i).path(leave_i1,1:2));
            leave_i1=leave_i1+1;
        end
        output.leave_i=leave_i1;
        output.leave=Agents(i).path(leave_i1,:);
        output.center=ActivatedCenters(j,1:2);
        output.isActivated=1; % is activated
        
        
        Agents(i).confliction=output;
        Agents(i).i=leave_i1; % near center, do control round, update i % new
        ActivatedCenters(j,3)=ActivatedCenters(j,3)+1;
        disp('checkNoActivatedCenter +1')
%         disp(i)

        
        outputSys=Sys;
        outputSys.Agents=Agents;
        outputSys.ActivatedCenters=ActivatedCenters;
        return;
    end
end

% no activated collision is found, then return
outputSys=Sys;
outputSys.Agents=Agents;

end