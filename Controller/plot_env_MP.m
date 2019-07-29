function plot_env_MP(env,i)
    figure(i)
    x_max=env.x_max;
    y_max=env.y_max;
    ob_vec=env.ob_vec;
    set(gcf,'unit','normalized','position',[0.4 0.4 0.5 0.5])
    axis equal
    axis([0 x_max 0 y_max])
    for j=1:size(ob_vec,1)
        obstacle=ob_vec(j,:);
        rectangle('Position',obstacle,'FaceColor',[0.7 0.7 0.7],'EdgeColor',[1 1 1])
        hold on
    end
end