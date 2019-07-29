clearvars
close all
rand('state',1);

%% env
% env.x_max = 500;
% env.y_max = 500;
% x_max=500;
% y_max=500;
% width_c=50;
% width_b=100;
% l_block=0.5*(x_max-width_c);
% h_block=(y_max-2*width_b);
% obstacle1 = [0,width_b,l_block,h_block];
% obstacle2 = [x_max-l_block,width_b,l_block,h_block];
% ob_vec=[obstacle1;obstacle2];
% env.ob_vec=ob_vec;
env_plot.x_max = 480;
env_plot.y_max = 440;
env_MP.x_max = 480;
env_MP.y_max = 440;
x_max=480;
y_max=440;

obstacle1 = [0,85,195, 90];
obstacle2 = [285, 85, 195, 90];
obstacle3 = [0,265,195, 90];
obstacle4 = [285, 265, 195, 90];
ob_vec_plot=[obstacle1;obstacle2;obstacle3;obstacle4];
env_plot.ob_vec=ob_vec_plot;

obstacle1 = [0,60,220, 140];
obstacle2 = [260, 60, 220, 140];
obstacle3 = [0,240,220, 140];
obstacle4 = [260, 240, 220, 140];
ob_vec_MP=[obstacle1;obstacle2;obstacle3;obstacle4];
env_MP.ob_vec=ob_vec_MP;

%% constant
EPS = 10;
rho=40;
    BETA=EPS/rho; 
numNodes=2000;

% 7-10 deconfliciton
dt=0.1;

confliction_init.notCollided=1;

%% Agent1
agent1.q_start.coord = [20 40];
agent1.q_start.dir=0; 
agent1.q_start.cost = 0;
agent1.q_start.parent = 0;

agent1.q_goal.coord = [460 420];
agent1.q_goal.cost = 0;
agent1.q_goal.dir=0;

agent1.speedmode=0;
agent1.q_ci=agent1.q_start;

agent1.i=2; %used for path following

agent1.confliction=confliction_init;


%% Agent2
agent2.q_start.coord = [460  40];
agent2.q_start.dir=pi-0.02; 
agent2.q_start.cost = 0;
agent2.q_start.parent = 0;

agent2.q_goal.coord = [150 400];
agent2.q_goal.cost = 0;
agent2.q_goal.dir=-pi+0.02;

agent2.speedmode=0;
agent2.q_ci=agent2.q_start;

agent2.i=2;

agent2.confliction=confliction_init;

%% Agent3
agent3.q_start.coord = [100  230];
agent3.q_start.dir=0; 
agent3.q_start.cost = 0;
agent3.q_start.parent = 0;

agent3.q_goal.coord = [380 210];
agent3.q_goal.cost = 0;
agent3.q_goal.dir=0;

agent3.speedmode=0;
agent3.q_ci=agent3.q_start;

agent3.i=2;

agent3.confliction=confliction_init;

%% Agent4
agent4.q_start.coord = [380  230];
agent4.q_start.dir=pi-0.01; 
agent4.q_start.cost = 0;
agent4.q_start.parent = 0;

agent4.q_goal.coord = [100 210];
agent4.q_goal.cost = 0;
agent4.q_goal.dir=pi-0.01;

agent4.speedmode=0;
agent4.q_ci=agent4.q_start;

agent4.i=2;

agent4.confliction=confliction_init;

% load('path')
% agent1.path=newpath;
% agent1.i=2;

% load('path_data')
load('newpath_data_simulation')
agent1.path=newpath1;
agent2.path=newpath2;
agent3.path=newpath3;
agent4.path=newpath4;


%% plot environment and path
plot_env_MP(env_MP,1)
hold on
plot_env(env_plot,1)
plot(agent1.path(:,1),agent1.path(:,2),'--', 'Color', [1 0 0]*0.75, 'LineWidth',2.5)
plot(agent2.path(:,1),agent2.path(:,2),'--', 'Color', [1 1 0]*0.75,'LineWidth',2.5)
plot(agent3.path(:,1),agent3.path(:,2),'--', 'Color', [0 1 1]*0.75,'LineWidth',2.5)
plot(agent4.path(:,1),agent4.path(:,2),'--', 'Color', [1 0 1]*0.75,'LineWidth',2.5)
axis equal
axis([0 x_max 0 y_max])

clr=[ 1 0 0; 1 1 0; 0 1 0; 1 0 1];

% state_check1=[];
% state_check2=[];
dist_agent=[];

Agents=[agent1; agent2; agent3; agent4];

%% System construction
Sys.Agents=Agents;
Sys.ActivatedCenters=[];

%% Simulation
for t=0:dt:80
    disp(t)
    %%
%     for i=1:4
%         Sys.Agents(i)=controlFollowPath(Sys.Agents(i),rho,EPS,dt);
%     end

    %% deconfliciton
    Sys=controlSystem3(Sys,rho,EPS,dt);
    
    
    Agents=Sys.Agents;
    % --- show the states
    for i=1:4
        disp(Sys.Agents(i).confliction)
    end
    disp(Sys.ActivatedCenters)
    % ---
    
    for i=1:4
        
        plot(Agents(i).q_ci.coord(1),Agents(i).q_ci.coord(2),'.','Color',clr(i,:),'LineWidth',1)
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
    
    dist_agent=[dist_agent [norm(Agents(1).q_ci.coord-Agents(2).q_ci.coord); norm(Agents(2).q_ci.coord-Agents(3).q_ci.coord); norm(Agents(3).q_ci.coord-Agents(4).q_ci.coord); norm(Agents(4).q_ci.coord-Agents(1).q_ci.coord); norm(Agents(1).q_ci.coord-Agents(3).q_ci.coord); norm(Agents(2).q_ci.coord-Agents(4).q_ci.coord)]];
end
figure(1)
xlabel('x')
ylabel('y')
% figure(2)
% plot(state_check1)
% hold on
% plot(state_check2,'--')

figure(3)
for i=1:6
    plot(0:dt:80,dist_agent(i,:),'LineWidth',1.5)
    hold on
end
xlabel('t(sec)')
ylabel('Distance')
