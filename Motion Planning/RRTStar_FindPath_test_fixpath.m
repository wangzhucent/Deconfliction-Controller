

EPS=10;
step=7;
rho=40;
newpath=fixPath(path, rho, step);
figure(5)
plot_env(env,5)
hold on
plot(path(:,1),path(:,2),'bv')
hold on
plot(newpath(:,1),newpath(:,2),'k.')