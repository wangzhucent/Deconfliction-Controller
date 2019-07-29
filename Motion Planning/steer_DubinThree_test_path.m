EPS=10;
step=1;
rho=40;
newpath=fixPath(path, rho, step);
figure(5)
% plot(path(:,1),path(:,2),'bv')
% hold on
% plot(newpath(:,1),newpath(:,2),'r.')
q6.coord=path(6,1:2);
q6.dir=path(6,3);
q7.coord=path(7,1:2);
q7.dir=path(7,3);

for i=1:100
    tempnode=steer_DubinThree(q6,q7,rho,step);
    plot(tempnode(1),tempnode(2),'g.')
    hold on
    q6.coord=tempnode;
    q6.dir=steer_DubinThree2(q6,q7,rho,step);
end

