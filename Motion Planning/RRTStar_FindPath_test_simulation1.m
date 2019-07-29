clearvars
close all
rand('state',1);


%% env 
env_plot.x_max = 480;
env_plot.y_max = 440;
env_MP.x_max = 480;
env_MP.y_max = 440;
x_max=480;
y_max=440;

obstacle1 = [0,80,200, 100];
obstacle2 = [280, 80, 200, 100];
obstacle3 = [0,260,200, 100];
obstacle4 = [280, 260, 200, 100];
ob_vec_plot=[obstacle1;obstacle2;obstacle3;obstacle4];
env_plot.ob_vec=ob_vec_plot;

obstacle1 = [0,60,220, 140];
obstacle2 = [260, 60, 220, 140];
obstacle3 = [0,240,220, 140];
obstacle4 = [260, 240, 220, 140];
ob_vec_MP=[obstacle1;obstacle2;obstacle3;obstacle4];
env_MP.ob_vec=ob_vec_MP;


%% constant definition
EPS = 10;
rho=40;
    BETA=EPS/rho; 
numNodes=2000;

%% Agent1
agent1.q_start.coord = [20 40];
agent1.q_start.dir=0; 
agent1.q_start.cost = 0;
agent1.q_start.parent = 0;

agent1.q_goal.coord = [460 420];
agent1.q_goal.cost = 0;
agent1.q_goal.dir=0;
% agent1.nodes(1) =agent1.q_start;

% agent1.q_ci=agent1.q_start.coord;
% agent1.phi_ci=agent1.q_start.dir;
% agent1.i=1;
% agent1.confliction.state=zeros(num_agent,1);
% agent1.confliction.agents=[];

%% Agent2
agent2.q_start.coord = [460  40];
agent2.q_start.dir=pi-0.02; 
agent2.q_start.cost = 0;
agent2.q_start.parent = 0;

agent2.q_goal.coord = [150 400];
agent2.q_goal.cost = 0;
agent2.q_goal.dir=-pi+0.02;

%% Agent3
agent3.q_start.coord = [100  230];
agent3.q_start.dir=0; 
agent3.q_start.cost = 0;
agent3.q_start.parent = 0;

agent3.q_goal.coord = [380 210];
agent3.q_goal.cost = 0;
agent3.q_goal.dir=0;

%% Agent4
agent4.q_start.coord = [380  230];
agent4.q_start.dir=pi-0.01; 
agent4.q_start.cost = 0;
agent4.q_start.parent = 0;

agent4.q_goal.coord = [100 210];
agent4.q_goal.cost = 0;
agent4.q_goal.dir=pi-0.01;

plot_env(env_plot,1);
hold on
plot(agent1.q_goal.coord(1),agent1.q_goal.coord(2),'ro','LineWidth',2)
plot(agent2.q_goal.coord(1),agent2.q_goal.coord(2),'ro','LineWidth',2)
plot(agent3.q_goal.coord(1),agent3.q_goal.coord(2),'ro','LineWidth',2)
plot(agent4.q_goal.coord(1),agent3.q_goal.coord(2),'ro','LineWidth',2)
 
path1=RRTStar_FindPath(agent1,EPS,rho,numNodes,env_MP);
path2=RRTStar_FindPath(agent2,EPS,rho,numNodes,env_MP);
path3=RRTStar_FindPath(agent3,EPS,rho,numNodes,env_MP);
path4=RRTStar_FindPath(agent4,EPS,rho,numNodes,env_MP);
newpath1=fixPath(path1,rho,7);
newpath2=fixPath(path2,rho,7);
newpath3=fixPath(path3,rho,7);
newpath4=fixPath(path4,rho,7);

save('newpath_data_simulation','newpath1','newpath2','newpath3','newpath4')
plot(newpath1(:,1),newpath1(:,2),'y.')  
plot(newpath2(:,1),newpath2(:,2),'b.') 
plot(newpath3(:,1),newpath3(:,2),'g.') 
plot(newpath4(:,1),newpath4(:,2),'m.') 
plot(path1(:,1),path1(:,2),'y--')  
plot(path2(:,1),path2(:,2),'b--') 
plot(path3(:,1),path3(:,2),'g--') 
plot(path4(:,1),path4(:,2),'m--') 



