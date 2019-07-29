function output=controlFollowPath(agent,rho,eps,dt)
% output=agent;
i=agent.i;
path=agent.path;
i_ref=agent.i+3;
if i_ref>size(agent.path,1)
    output=agent;
    return;
end
q_ref=agent.path(i_ref,:);
alpha_ref=atan2(q_ref(2)-agent.q_ci.coord(2),q_ref(1)-agent.q_ci.coord(1));

alpha=agent.q_ci.dir;
diff=alpha_ref-alpha;
if diff>pi
    diff=diff-2*pi;
elseif diff<-pi
    diff=diff+2*pi;
end

if abs(diff)>eps/rho*dt
    alpha=alpha+sign(diff)*eps/rho*dt;
else
    alpha=alpha_ref;
end


if alpha>pi
    alpha=alpha-2*pi;
elseif diff<-pi
    alpha=alpha+2*pi;
end

if norm(agent.q_ci.coord-path(i-1,1:2))>norm(agent.q_ci.coord-path(i,1:2))
    agent.i=i+1;
end
agent.q_ci.coord=agent.q_ci.coord+(eps+eps*0.1*agent.speedmode)*dt*[cos(alpha) sin(alpha)];
agent.q_ci.dir=alpha;

output=agent;

end