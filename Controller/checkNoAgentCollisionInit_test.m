clearvars
close all
rand('state',1);

env.x_max = 500;
env.y_max = 500;
x_max=500;
y_max=500;
width_c=50;
width_b=100;
l_block=0.5*(x_max-width_c);
h_block=(y_max-2*width_b);
obstacle1 = [0,width_b,l_block,h_block];
obstacle2 = [x_max-l_block,width_b,l_block,h_block];
ob_vec=[obstacle1;obstacle2];
env.ob_vec=ob_vec;

EPS = 10;
rho=40;
    BETA=EPS/rho; 
numNodes=2000;

% 7-10 deconfliciton
dt=0.1;

%% Agent1
agent1.q_start.coord = [20 20];
agent1.q_start.dir=pi/4; 
agent1.q_start.cost = 0;
agent1.q_start.parent = 0;

agent1.q_goal.coord = [x_max-20 y_max-20];
agent1.q_goal.cost = 0;
agent1.q_goal.dir=0;

agent1.speedmode=0;
agent1.q_ci=agent1.q_start;

agent1.i=2; %used for path following

% center=[100 20];


%% Agent2
agent2.q_start.coord = [x_max-20 y_max-20];
agent2.q_start.dir=-3*pi/4; 
agent2.q_start.cost = 0;
agent2.q_start.parent = 0;

agent2.q_goal.coord = [30 20];
agent2.q_goal.cost = 0;
agent2.q_goal.dir=-2*pi/3;

agent2.speedmode=0;
agent2.q_ci=agent2.q_start;

agent2.i=2;

% load('path')
% agent1.path=newpath;
% agent1.i=2;

% load('path_data')
load('newpath_data')
agent1.path=newpath1;
agent2.path=newpath2;


plot(agent1.path(:,1),agent1.path(:,2),'rv')
hold on
plot(agent2.path(:,1),agent2.path(:,2),'yv')
axis equal
% axis([10 50 10 50])

state_check1=[];
state_check2=[];
dist_agent=[];
for t=0:dt:100
    agent1=controlFollowPath(agent1,rho,EPS,dt);
%     agent1.i
    plot(agent1.q_ci.coord(1),agent1.q_ci.coord(2),'b.')
    agent2=controlFollowPath(agent2,rho,EPS,dt);
%     agent1.i
    plot(agent2.q_ci.coord(1),agent2.q_ci.coord(2),'b.')
    drawnow
%     pause(0.5)
    [temp1,temp2]=checkNoAgentCollisionInit(agent1,agent2);
    state_check1=[state_check1 temp1.notCollided];
    state_check2=[state_check2 temp2.notCollided];
    if ~temp1.notCollided
        plot(temp1.approach(1,1),temp1.approach(1,2),'k.')
        plot(temp1.leave(1,1),temp1.leave(1,2),'k.')
        plot(temp2.approach(1,1),temp2.approach(1,2),'k.')
        plot(temp2.leave(1,1),temp2.leave(1,2),'k.')
        plot(temp1.center(1,1),temp1.center(1,2),'ko','LineWidth',1.5)
    end
        
    dist_agent=[dist_agent norm(agent1.q_ci.coord-agent2.q_ci.coord)];
    hold on;
end

figure(2)
plot(state_check1)
hold on
plot(state_check2,'--')

figure(3)
plot(dist_agent)
