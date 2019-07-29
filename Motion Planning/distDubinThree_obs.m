function d=distDubinThree_obs(node1,node2,rho,ob_vec)
p1=node1.coord;
alpha1=node1.dir;

p2=node2.coord;
alpha2=node2.dir;

[m,n]=size(ob_vec);
for i=1:m
    if ~noCollision(p1,p2,ob_vec(i,:))
        d=inf;
        return;
    end
end



radius=rho;
center1L=zeros(2,1);
center1R=zeros(2,1);
center2L=zeros(2,1);
center2R=zeros(2,1);

% if norm(p1-p2)<rho
%     d=inf;
%     return;
% end

    center1L(1)=p1(1)+radius*cos(alpha1+pi/2);
    center1L(2)=p1(2)+radius*sin(alpha1+pi/2);
    
    center1R(1)=p1(1)+radius*cos(alpha1-pi/2);
    center1R(2)=p1(2)+radius*sin(alpha1-pi/2);
    
    center2L(1)=p2(1)+radius*cos(alpha2+pi/2);
    center2L(2)=p2(2)+radius*sin(alpha2+pi/2);
    
    center2R(1)=p2(1)+radius*cos(alpha2-pi/2);
    center2R(2)=p2(2)+radius*sin(alpha2-pi/2);
    
% 1-LSL
lineDist=norm(center1L-center2L);
%     disp(lineDist);
if lineDist<0.00001
    dLSL=radius*(alpha2-alpha1+2*pi*(alpha2-alpha1<-0.0001));
else
    % disp(radius)
%     disp(p1)
%     disp(p2)
%     disp(center2L)
%     disp(center1L)
    theta=atan2(center2L(2)-center1L(2),center2L(1)-center1L(1));
    dtheta1=angle_normalize(theta-alpha1);
    dtheta2=angle_normalize(alpha2-theta);
    cirDist=abs(dtheta1+2*pi*(dtheta1<-0.00001))*radius+abs(dtheta2+2*pi*(dtheta2<-0.00001))*radius;
    dLSL=lineDist+cirDist;
end
% 2-LSR
lineDist=norm(center1L-center2R);

if lineDist>radius*2
    ddtheta=asin(radius/lineDist*2);
    theta=atan2(center2R(2)-center1L(2),center2R(1)-center1L(1))+ddtheta;

    dtheta1=angle_normalize(theta-alpha1);
    dtheta2=angle_normalize(alpha2-theta);
    cirDist=abs(dtheta1+2*pi*(dtheta1<-0.00001))*radius+abs(dtheta2-2*pi*(dtheta2>0.00001))*radius;
    dLSR=lineDist*cos(ddtheta)+cirDist;
%     data(2,:)=[dtheta1 theta dtheta2 abs(dtheta1+2*pi*(dtheta1<0))*radius lineDist*cos(ddtheta) abs(dtheta2-2*pi*(dtheta2>0))*radius];
else
    dLSR=inf;
%     data(2,:)=zeros(1,6);
%     data(2,5)=inf;
end

% 3-RSL
lineDist=norm(center1R-center2L);

if lineDist>2*radius
    ddtheta=asin(radius/lineDist*2);
    theta=atan2(center2L(2)-center1R(2),center2L(1)-center1R(1))-ddtheta;

    dtheta1=angle_normalize(theta-alpha1);
    dtheta2=angle_normalize(alpha2-theta);
    cirDist=abs(dtheta1-2*pi*(dtheta1>0.00001))*radius+abs(dtheta2+2*pi*(dtheta2<-0.00001))*radius;
    dRSL=lineDist*cos(ddtheta)+cirDist;

%     data(3,:)=[dtheta1 theta dtheta2 abs(dtheta1-2*pi*(dtheta1>0))*radius lineDist*cos(ddtheta) abs(dtheta2+2*pi*(dtheta2<0))*radius];
else
    dRSL=inf;
%     data(3,:)=zeros(1,6);
%     data(3,5)=inf;
end

% 4-RSR
lineDist=norm(center1R-center2R);
if lineDist<0.00001
    dRSR=radius*(alpha1-alpha2+2*pi*(alpha1-alpha2<-0.0001));
else
    theta=atan2(center2R(2)-center1R(2),center2R(1)-center1R(1));
    dtheta1=angle_normalize(theta-alpha1);
    dtheta2=angle_normalize(alpha2-theta);
    cirDist=abs(dtheta1-2*pi*(dtheta1>0.00001))*radius+abs(dtheta2-2*pi*(dtheta2>0.00001))*radius;
    dRSR=lineDist+cirDist;
end
% [dLSL, dLSR, dRSL, dRSR]
[d,n_dubin]=min([dLSL, dLSR, dRSL, dRSR]);
% disp(n_dubin)
end