function output=controlAround(agent,center,rho,eps,dt)
K=1;

x0=center(1);
y0=center(2);
xc=agent.q_ci.coord(1);
yc=agent.q_ci.coord(2);
x=xc-x0;
y=yc-y0;

alpha_ref=atan2(x/rho+y/rho.*(1-(x/rho).^2-(y/rho).^2),-y/rho+x/rho.*(1-(x/rho).^2-(y/rho).^2) );

    if alpha_ref>pi
        alpha_ref=alpha_ref-2*pi;
    elseif alpha_ref<-pi
        alpha_ref=alpha_ref+2*pi;
    end

alpha=agent.q_ci.dir;

diff=alpha_ref-alpha;
if diff>pi
    diff=diff-2*pi;
elseif diff<-pi
    diff=diff+2*pi;
end

if abs(diff)>eps/rho*dt
    alpha=eps/rho*dt*sign(diff)+alpha;
else
    alpha=alpha_ref;
end

if alpha>pi
    alpha=alpha-2*pi;
elseif alpha<-pi
    alpha=alpha+2*pi;
end

% (eps+0.1*eps*agent.speedmode)*dt
agent.q_ci.coord=agent.q_ci.coord+(eps+0.1*eps*agent.speedmode)*dt*[cos(alpha) sin(alpha)];
agent.q_ci.dir=alpha;
output=agent;
end