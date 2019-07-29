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
h_block=(y_max-2*width_b)
obstacle1 = [0,width_b,l_block,h_block];
obstacle2 = [x_max-l_block,width_b,l_block,h_block];
ob_vec=[obstacle1;obstacle2];
env.ob_vec=ob_vec;
plot_env(env,1)
