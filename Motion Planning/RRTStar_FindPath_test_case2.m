clearvars
close all
rand('state',1);


%% env 
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

%% constant definition
EPS = 10;
rho=40;
    BETA=EPS/rho; 
numNodes=1000;

%% Agent1
agent1.q_start.coord = [20 20];
agent1.q_start.dir=pi/4; 
agent1.q_start.cost = 0;
agent1.q_start.parent = 0;

agent1.q_goal.coord = [x_max-20 y_max-20];
agent1.q_goal.cost = 0;
agent1.q_goal.dir=0;
% agent1.nodes(1) =agent1.q_start;

% agent1.q_ci=agent1.q_start.coord;
% agent1.phi_ci=agent1.q_start.dir;
% agent1.i=1;
% agent1.confliction.state=zeros(num_agent,1);
% agent1.confliction.agents=[];


%% Agent2
agent2.q_start.coord = [x_max-20 20];
agent2.q_start.dir=-3*pi/4; 
agent2.q_start.cost = 0;
agent2.q_start.parent = 0;

agent2.q_goal.coord = [x_max-20 y_max-20];
agent2.q_goal.cost = 0;
agent2.q_goal.dir=0;

plot_env(env,1);
hold on
plot(agent1.q_goal.coord(1),agent1.q_goal.coord(2),'ro','LineWidth',2)
plot(agent2.q_goal.coord(1),agent2.q_goal.coord(2),'ro','LineWidth',2)
 
path1=RRTStar_FindPath(agent1,EPS,rho,numNodes,env);
path2=RRTStar_FindPath(agent2,EPS,rho,numNodes,env);
newpath1=fixPath(path1,rho,7);
newpath2=fixPath(path2,rho,7);

save('newpath_data2','newpath1','newpath2')
plot(newpath1(:,1),newpath1(:,2),'y.')  
plot(newpath2(:,1),newpath2(:,2),'b.') 
plot(path1(:,1),path1(:,2),'g--')  
plot(path2(:,1),path2(:,2),'m--') 

plot(newpath1(:,1),newpath1(:,2),'y--')  
plot(newpath2(:,1),newpath2(:,2),'p--') 


