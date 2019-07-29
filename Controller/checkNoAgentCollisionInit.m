function [output1, output2]=checkNoAgentCollisionInit(agent1,agent2)
    rho=40;
    
    output1.notCollided=1;
    output2.notCollided=1;
    path1=agent1.path;
    i1=agent1.i;
    l1=norm(path1(i1,1:2)-agent1.q_ci.coord);
    
    path2=agent2.path;
    i2=agent2.i;
    l2=norm(path2(i2,1:2)-agent2.q_ci.coord);
    
    while (l1<6*40 && l2<6*40) && i1<size(agent1.path,1) && i2<size(agent2.path,1)
        if norm(path1(i1,1:2)-path2(i2,1:2))<15            
            %% deal with the collision parameters
            output1.notCollided=0;
            output2.notCollided=0;
            approach1=sum(path1(i1-3:i1-1,:))/3;
            approach2=sum(path2(i2-3:i2-1,:))/3;

            diff=(approach1(3)-approach2(3));
            if diff>pi
                diff=diff-2*pi;
            elseif diff<-pi
                diff=diff+2*pi;
            end

            if abs(diff)>pi/4
                output1.isAround=1;
                output2.isAround=1;
            else
                output1.isAround=0;
                output2.isAround=0;
            end

            leave_i1=i1;
            leave_l1=l1;
            leave_i2=i2;
            leave_l2=l2;

            while leave_l1-l1<2*rho && leave_i1<size(path1,1)
                leave_l1=leave_l1+norm(path1(leave_i1+1,1:2)-path1(leave_i1,1:2));
                leave_i1=leave_i1+1;
            end

            while leave_l2-l2<2*rho && leave_i2<size(path2,1)
                leave_l2=leave_l2+norm(path2(leave_i2+1,1:2)-path2(leave_i2,1:2));
                leave_i2=leave_i2+1;
            end
            
            diff=atan2(path1(i1,2)-path2(i2,2), path1(i1,1)-path2(i2,1))-path1(i1,3);
            if diff>pi
                diff=diff-2*pi;
            elseif diff<-pi
                diff=diff+2*pi;
            end
            
            if abs(diff)<pi/2 
                output1.speedmode=1;
                output2.speedmode=-1;
            else
                output1.speedmode=-1;
                output2.speedmode=1;
            end

            output1.approach=approach1; 
            output2.approach=approach2;
            output1.leave=path1(leave_i1,:); 
            output2.leave=path2(leave_i2,:);
            output1.leave_i=leave_i1;
            output2.leave_i=leave_i2;
            output1.center=0.5*(agent1.path(i1,1:2)+agent2.path(i2,1:2)); 
            output2.center=0.5*(agent1.path(i1,1:2)+agent2.path(i2,1:2)); 
            return;
        end
        if l1<=l2
            i1=i1+1;
            l1=l1+norm(path1(i1-1,1:2)-path1(i1,1:2));
        else
            i2=i2+1;
            l2=l2+norm(path2(i2-1,1:2)-path2(i2,1:2));
        end
        
    end
    
    
    
    output1.notCollided=1;
    output2.notCollided=1;
    
        
    return;
    
    
end