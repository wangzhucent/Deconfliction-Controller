load simulation3_data_original
dist_agent1=dist_agent;
load simulation3_data_deconfliction
dist_agent2=dist_agent;

dt=0.1;
t=0:dt:80;
figure(3)
for i=[1 3]
    if i==1
        clr=[1 0.2 0];
    elseif i==3
        clr=[0 1 0.1];
    else
        clr=rand(1,3);
    end
    plot(t,dist_agent1(i,:),'--','Color',clr,'LineWidth',1.5);
    hold on
    plot(t,dist_agent2(i,:),'Color',clr,'LineWidth',1.5);
end
legend('Distance between Agent_1 and Agnet_2 without deconfliction', 'Distance between Agent_1 and Agnet_2 with deconfliction', 'Distance between Agent_3 and Agnet_4 without deconfliction', 'Distance between Agent_3 and Agnet_4 with deconfliction' )
xlabel('t(sec)')
ylabel('Distance')