
load('path_data0')
EPS=10;
step=6;
rho=40;
newpath1=fixPath(path1, rho, step);
newpath2=fixPath(path2, rho, step);
save('newpath_data','newpath1','newpath2')
figure(5)
plot_env(env,5)
hold on
plot(path2(:,1),path2(:,2),'bv')
hold on
plot(newpath2(:,1),newpath2(:,2),'k.')