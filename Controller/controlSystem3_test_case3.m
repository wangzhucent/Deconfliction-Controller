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
agent1.q_start.coord = [250 200];
agent1.q_start.dir=pi/2; 
agent1.q_start.cost = 0;
agent1.q_start.parent = 0;

agent1.q_goal.coord = [x_max-20 y_max-20];
agent1.q_goal.cost = 0;
agent1.q_goal.dir=0;

agent1.speedmode=0;
agent1.q_ci=agent1.q_start;

agent1.i=2; %used for path following

agent1.confliction=confliction_init;


%% Agent2
agent2.q_start.coord = [100 y_max-20];
agent2.q_start.dir=0; 
agent2.q_start.cost = 0;
agent2.q_start.parent = 0;

agent2.q_goal.coord = [250 20];
agent2.q_goal.cost = 0;
agent2.q_goal.dir=-pi/2;

agent2.speedmode=0;
agent2.q_ci=agent2.q_start;

agent2.i=2;

agent2.confliction=confliction_init;

% load('path')
% agent1.path=newpath;
% agent1.i=2;

% load('path_data')
load('newpath_data3')
agent1.path=newpath1;
agent2.path=newpath2;


% plot_env(env,1)
plot(agent1.path(:,1),agent1.path(:,2),'rv')
hold on
plot(agent2.path(:,1),agent2.path(:,2),'yv')
axis equal
% axis([10 50 10 50])

% state_check1=[];
% state_check2=[];
dist_agent=[];

Agents=[agent1; agent2];

%% System construction
Sys.Agents=Agents;
Sys.ActivatedCenters=[];

%% Simulation
for t=0:dt:80
    disp(t)
%     Agents=control(Agents,rho,EPS,dt);
    Sys=controlSystem3(Sys,rho,EPS,dt);
    
    Agents=Sys.Agents;
    % --- show the states
    for i=1:2
        disp(Sys.Agents(i).confliction)
    end
    disp(Sys.ActivatedCenters)
    % ---
    
    for i=1:2
        plot(Agents(i).q_ci.coord(1),Agents(i).q_ci.coord(2),'b.')
        hold on
        temp=Agents(i);
        plot(Agents(i).path(temp.i,1), Agents(i).path(temp.i,2),'g.')
        if ~Agents(i).confliction.notCollided
            plot(Agents(i).confliction.center(1), Agents(i).confliction.center(2),'ko')
            hold on
            plot(Agents(i).confliction.leave(1), Agents(i).confliction.leave(2),'k.')
        end
        drawnow
    end    
    
    dist_agent=[dist_agent norm(Agents(1).q_ci.coord-Agents(2).q_ci.coord)];
end
figure(3)
plot(0:dt:80,dist_agent)
