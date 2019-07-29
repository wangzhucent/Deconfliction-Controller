function d=distDubinThree(node1,node2,rho)
p1=node1.coord;
alpha1=node1.dir;

p2=node2.coord;
alpha2=node2.dir;

radius=rho;
center1L=zeros(2,1);
center1R=zeros(2,1);
center2L=zeros(2,1);
center2R=zeros(2,1);

if norm(p1-p2)<rho
    d=inf;
    return;
end

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
theta=atan2(center2L(2)-center1L(2),center2L(1)-center1L(1));
% disp(center2L)
% disp(center1L)
dtheta1=angle_normalize(theta-alpha1);
dtheta2=angle_normalize(alpha2-theta);
cirDist=abs(dtheta1+2*pi*(dtheta1<0))*radius+abs(dtheta2+2*pi*(dtheta2<0))*radius;
dLSL=lineDist+cirDist;

% 2-LSR
lineDist=norm(center1L-center2R);

if lineDist>radius*2
    ddtheta=asin(radius/lineDist*2);
    theta=atan2(center2R(2)-center1L(2),center2R(1)-center1L(1))+ddtheta;

    dtheta1=angle_normalize(theta-alpha1);
    dtheta2=angle_normalize(alpha2-theta);
    cirDist=abs(dtheta1+2*pi*(dtheta1<0))*radius+abs(dtheta2-2*pi*(dtheta2>0))*radius;
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
    cirDist=abs(dtheta1-2*pi*(dtheta1>0))*radius+abs(dtheta2+2*pi*(dtheta2<0))*radius;
    dRSL=lineDist*cos(ddtheta)+cirDist;

%     data(3,:)=[dtheta1 theta dtheta2 abs(dtheta1-2*pi*(dtheta1>0))*radius lineDist*cos(ddtheta) abs(dtheta2+2*pi*(dtheta2<0))*radius];
else
    dRSL=inf;
%     data(3,:)=zeros(1,6);
%     data(3,5)=inf;
end

% 4-RSR
lineDist=norm(center1R-center2R);
theta=atan2(center2R(2)-center1R(2),center2R(1)-center1R(1));
dtheta1=angle_normalize(theta-alpha1);
dtheta2=angle_normalize(alpha2-theta);
cirDist=abs(dtheta1-2*pi*(dtheta1>0))*radius+abs(dtheta2-2*pi*(dtheta2>0))*radius;
dRSR=lineDist+cirDist;

[d,n]=min([dLSL, dLSR, dRSL, dRSR]);
% disp(n);
end