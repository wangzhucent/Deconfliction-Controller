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

confliction_init.notCollided=1;

%% Agent1
agent1.q_start.coord = [150 40];
agent1.q_start.dir=0; 
agent1.q_start.cost = 0;
agent1.q_start.parent = 0;

agent1.q_goal.coord = [20 420];
agent1.q_goal.cost = 0;
agent1.q_goal.dir=pi-0.02;

agent1.speedmode=0;
agent1.q_ci=agent1.q_start;

agent1.i=2; %used for path following

agent1.confliction=confliction_init;


%% Agent2
agent2.q_start.coord = [20  40];
agent2.q_start.dir=0; 
agent2.q_start.cost = 0;
agent2.q_start.parent = 0;

agent2.q_goal.coord = [360 220];
agent2.q_goal.cost = 0;
agent2.q_goal.dir=0;

agent2.speedmode=0;
agent2.q_ci=agent2.q_start;

agent2.i=2;

agent2.confliction=confliction_init;

%% Agent3
agent3.q_start.coord = [20  220];
agent3.q_start.dir=0; 
agent3.q_start.cost = 0;
agent3.q_start.parent = 0;

agent3.q_goal.coord = [460 220];
agent3.q_goal.cost = 0;
agent3.q_goal.dir=0;

agent3.speedmode=0;
agent3.q_ci=agent3.q_start;

agent3.i=2;

agent3.confliction=confliction_init;

% load('path')
% agent1.path=newpath;
% agent1.i=2;

% load('path_data')
load('newpath_data_complex')
agent1.path=newpath1;
agent2.path=newpath2;
agent3.path=newpath3;


% plot_env(env,1)
plot(agent1.path(:,1),agent1.path(:,2),'rv')
hold on
plot(agent2.path(:,1),agent2.path(:,2),'yv')
plot(agent3.path(:,1),agent3.path(:,2),'gv')
axis equal
% axis([10 50 10 50])

% state_check1=[];
% state_check2=[];
dist_agent=[];

Agents=[agent1; agent2; agent3];



for t=0:dt:80
    t
%     agent1=controlFollowPath(agent1,rho,EPS,dt);
%     agent2=controlFollowPath(agent2,rho,EPS,dt);

%--------
    Agents=control(Agents,rho,EPS,dt);
    for i=1:3
        plot(Agents(i).q_ci.coord(1),Agents(i).q_ci.coord(2),'b.')
        hold on
        temp=Agents(i);
        plot(Agents(i).path(temp.i,1), Agents(i).path(temp.i,2),'g.')
        if ~Agents(i).confliction.notCollided
            plot(Agents(i).confliction.center(1), Agents(i).confliction.center(2),'ko')
            plot(Agents(i).confliction.leave(1), Agents(i).confliction.leave(2),'k.')
            
        end
        drawnow
    end
%     [Agents(1).speedmode Agents(2).speedmode]
    %-----------
%     agent1.i
%     plot(agent1.q_ci.coord(1),agent1.q_ci.coord(2),'b.')
%     agent1.i
%     plot(agent2.q_ci.coord(1),agent2.q_ci.coord(2),'b.')
%     drawnow
% %     pause(0.5)
%     [temp1,temp2]=checkNoAgentCollisionInit(agent1,agent2);
%     state_check1=[state_check1 temp1.notCollided];
%     state_check2=[state_check2 temp2.notCollided];
%     if ~temp1.notCollided
%         plot(temp1.approach(1,1),temp1.approach(1,2),'k.')
%         plot(temp1.leave(1,1),temp1.leave(1,2),'k.')
%         plot(temp2.approach(1,1),temp2.approach(1,2),'k.')
%         plot(temp2.leave(1,1),temp2.leave(1,2),'k.')
%         plot(temp1.center(1,1),temp1.center(1,2),'ko','LineWidth',1.5)
%     end
        
    dist_agent=[dist_agent [norm(Agents(1).q_ci.coord-Agents(2).q_ci.coord); norm(Agents(1).q_ci.coord-Agents(3).q_ci.coord);norm(Agents(2).q_ci.coord-Agents(3).q_ci.coord)]];
%     hold on;
end


figure(3)
plot(0:dt:80,dist_agent)
