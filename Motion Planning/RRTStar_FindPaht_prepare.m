clearvars
close all
rand('state',1);
x_max = 500;
y_max = 500;
%obstacle = [500,150,200,200];
obstacle1 = [0,200,100,100];
obstacle2 = [200,200,200,100];
obstacle3 = [500,0,100,500];
obstacle4 = [0,500,500,100];
ob_vec=[obstacle1;obstacle2;obstacle3;obstacle4];
EPS = 10;
rho=80;
step=10; %which is EPS in old edition
    BETA=EPS/rho; %my addition
%numNodes = 3000;        
numNodes=2000;

q_start.coord = [10 10];
q_start.dir=pi/3; %My addition
q_start.cost = 0;
q_start.parent = 0;
q_goal.coord = [490 490];
q_goal.cost = 0;
q_goal.dir=0;

nodes(1) = q_start;
figure(1)
set(gcf,'unit','normalized','position',[0.4 0.4 0.5 0.5])
axis equal
axis([0 x_max 0 y_max])
%axis([100 110 100 110])

rectangle('Position',obstacle1,'FaceColor',[0 .5 .5])
hold on
rectangle('Position',obstacle2,'FaceColor',[0 .5 .5])
plot(q_goal.coord(1),q_goal.coord(2),'ro','LineWidth',2)



for i = 1:1:numNodes
    
    i
       
    % Break if goal node is already reached
%     IsReached=0;
%         %     for j = 1:1:length(nodes)
%         %         if dist(nodes(j).coord, q_goal.coord)<5
%         %             IsReached=1;
%         %             break
%         %         end
%         %     end
%     if IsReached
%         break;
%     end
    
    
    if rand(1)<0.3
%         disp('sampling: goal 1')
        q_rand=q_goal;
    else
%         disp('sampling: goal 0')
        q_rand.coord = [floor(rand(1)*x_max) floor(rand(1)*y_max)];
        q_rand.dir=(rand(1)-0.5)*2*pi;
    end

    % Pick the closest node from existing list to branch out from
    ndist = [];
    for j = 1:1:length(nodes)
        n = nodes(j);
        %new func
        tmp = distDubinThree_obs(n, q_rand, rho, ob_vec);
        ndist = [ndist tmp];
    end
    [val, idx] = min(ndist);
    q_near = nodes(idx);
    
    
    q_new.coord = steer_DubinThree(q_near, q_rand, rho, step);
    q_new.dir=steer_DubinThree2(q_near, q_rand, rho, step);

    
    %/* RG-RRT part: the q_goal is closer to q_near or q_new?
    % while(dist(q_rand.coord,q_near.coord)<dist(q_rand.coord,q_new.coord)) || ~noCollision(q_rand.coord, q_near.coord, obstacle1) || ~noCollision(q_new.coord, q_near.coord, obstacle1) || ~noCollision(q_rand.coord, q_near.coord, obstacle2) || ~noCollision(q_new.coord, q_near.coord, obstacle2) 
        
    %/* not RG-RRT    
    while ~noCollision_obs(q_rand.coord, q_near.coord, ob_vec) || ~noCollision_obs(q_new.coord, q_near.coord, ob_vec)  
%         plot(q_rand.coord(1), q_rand.coord(2), 'o', 'Color',  [1 0 0.7410])
        q_rand.coord = [floor(rand(1)*x_max) floor(rand(1)*y_max)];
        q_rand.dir=(rand(1)-0.5)*2*pi;

        % Pick the closest node from existing list to branch out from
        ndist = [];
        for j = 1:1:length(nodes)
            n = nodes(j);
            %jan 28 tmp = dist_Dubin(n, q_rand, BETA, EPS);
            tmp = distDubinThree_obs(n, q_rand, rho, ob_vec);
            ndist = [ndist tmp];
        end
        [val, idx] = min(ndist);
        q_near = nodes(idx);
        
        q_new.coord = steer_DubinThree(q_near, q_rand, rho, step);
        q_new.dir=steer_DubinThree2(q_near, q_rand, rho, step);
    end
%     plot(q_rand.coord(1), q_rand.coord(2), 'x', 'Color',  [0 0.4470 0.7410])
    %*/
    
    % rewiring process
    if noCollision_obs(q_rand.coord, q_near.coord, ob_vec) && noCollision_obs(q_new.coord, q_near.coord, ob_vec)
%         line([q_near.coord(1), q_new.coord(1)], [q_near.coord(2), q_new.coord(2)], 'Color', 'k', 'LineWidth', 2);
%         drawnow
        hold on
        q_new.cost = distDubinThree(q_near, q_new, rho) + q_near.cost;
        
        % Within a radius of r, find all existing nodes
        q_nearest = [];
        r = 100;
        neighbor_count = 1;
        
        % Initialize cost to currently known value
        q_min = q_near;
        C_min = q_new.cost; 
        
        for j=1:length(nodes)
            if noCollision_obs(nodes(j).coord, q_new.coord,ob_vec) && distDubinThree_obs(nodes(j),q_new, rho, ob_vec)<r
                q_nearest(neighbor_count).coord=nodes(j).coord;
                q_nearest(neighbor_count).dir=nodes(j).dir;
                q_nearest(neighbor_count).cost=nodes(j).cost;
                if nodes(j).cost+distDubinThree_obs(nodes(j),q_new, rho, ob_vec)<C_min-2
                    q_min=nodes(j);
                    C_min=nodes(j).cost+distDubinThree_obs(nodes(j),q_new, rho, ob_vec);
                end
                neighbor_count=neighbor_count+1;
            end
        end 
                
        
        
        
        % Update parent to least cost-from node % near q_near best one
        for j = 1:1:length(nodes)
           if nodes(j).coord == q_min.coord
               q_new.parent = j;
           end
        end
%         q_new.parent=idx; % old q_near
        q_new.cost=C_min; % add by Zhu Wang
%         line([q_min.coord(1),q_new.coord(1)], [q_min.coord(2),q_new.coord(2)],'Color','g');
        % Append to nodes
        nodes = [nodes q_new];
        
        for j=1:length(nodes)-1
            if noCollision_obs(nodes(j).coord, q_new.coord,ob_vec) && distDubinThree_obs(q_new,nodes(j), rho, ob_vec)<r
                if nodes(j).cost>C_min+distDubinThree_obs(q_new, nodes(j),rho,ob_vec)
                    nodes(j).parent=length(nodes);
                    nodes(j).cost=C_min+distDubinThree_obs(q_new, nodes(j),rho,ob_vec);
%                     line([nodes(j).coord(1),q_new.coord(1)], [nodes(j).coord(2),q_new.coord(2)],'Color','y');
                end
            end
        end
        
        
        
    end
    if mod(i,100)==1
        text=['i=',int2str(i-1)];
        title(text)
    end
%     if (mod(i,10)==1)
%         F((i-1)/10+1)=getframe(gcf);
%     end
end

D = [];
for j = 1:1:length(nodes)
    tmpdist = distDubinThree_obs(nodes(j), q_goal, rho,ob_vec);
    D = [D tmpdist];
end

%% Search backwards from goal to start to find the optimal least cost path
path=[q_goal.coord q_goal.dir];
[val, idx] = min(D);

q_final = nodes(idx);
q_goal.parent = idx;
q_end = q_goal;

totaldist=q_final.cost+distDubinThree_obs(q_final,q_goal,rho,ob_vec);
q_goal.cost=totaldist;
nodes = [nodes q_goal];
while q_end.parent ~= 0
    start = q_end.parent;
% %     disp(start)
%     plot(q_end.coord(1), q_end.coord(2),'bv')
%     line([q_end.coord(1), nodes(start).coord(1)], [q_end.coord(2), nodes(start).coord(2)], 'Color', 'r', 'LineWidth', 2);
%     drawnow;
%     hold on;
    q_end = nodes(start);
%     F=[F getframe(gcf)];
    
    path=[q_end.coord q_end.dir; path];
end
% plot(path(:,1),path(:,2),'k--');
