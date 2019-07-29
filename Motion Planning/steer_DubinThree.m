function A = steer_DubinThree(node1, node2, rho, step)
A=zeros(1,2);


p1=node1.coord;
alpha1=node1.dir;

p2=node2.coord;
alpha2=node2.dir;

radius=rho;
center1L=zeros(2,1);
center1R=zeros(2,1);
center2L=zeros(2,1);
center2R=zeros(2,1);

    center1L(1)=p1(1)+radius*cos(alpha1+pi/2);
    center1L(2)=p1(2)+radius*sin(alpha1+pi/2);
    
    center1R(1)=p1(1)+radius*cos(alpha1-pi/2);
    center1R(2)=p1(2)+radius*sin(alpha1-pi/2);
    
    center2L(1)=p2(1)+radius*cos(alpha2+pi/2);
    center2L(2)=p2(2)+radius*sin(alpha2+pi/2);
    
    center2R(1)=p2(1)+radius*cos(alpha2-pi/2);
    center2R(2)=p2(2)+radius*sin(alpha2-pi/2);
    
    
data=zeros(4,6);
% 1-LSL
lineDist=norm(center1L-center2L);
if lineDist<0.00001
    theta=angle_normalize((alpha2-alpha1+2*pi*(alpha2-alpha1<-0.0001))*0.5+alpha1);
    dtheta1=angle_normalize(theta-alpha1);
    dtheta2=angle_normalize(alpha2-theta);
    dLSL=radius*(alpha2-alpha1+2*pi*(alpha2-alpha1<-0.0001));
    data(1,:)=[dtheta1 theta dtheta2 0.5*dLSL 0 0.5*dLSL];
else
    theta=atan2(center2L(2)-center1L(2),center2L(1)-center1L(1));
    dtheta1=angle_normalize(theta-alpha1);
    dtheta2=angle_normalize(alpha2-theta);
    cirDist=abs(dtheta1+2*pi*(dtheta1<0))*radius+abs(dtheta2+2*pi*(dtheta2<0))*radius;
    dLSL=lineDist+cirDist;
    data(1,:)=[dtheta1 theta dtheta2 abs(dtheta1+2*pi*(dtheta1<0))*radius lineDist abs(dtheta2+2*pi*(dtheta2<0))*radius];
end
% 2-LSR
lineDist=norm(center1L-center2R);

if lineDist>radius*2
    ddtheta=asin(radius/lineDist*2);
    theta=atan2(center2R(2)-center1L(2),center2R(1)-center1L(1))+ddtheta;

    dtheta1=angle_normalize(theta-alpha1);
    dtheta2=angle_normalize(alpha2-theta);
    cirDist=abs(dtheta1+2*pi*(dtheta1<0))*radius+abs(dtheta2-2*pi*(dtheta2>0))*radius;
    dLSR=lineDist*cos(ddtheta)+cirDist;
    data(2,:)=[dtheta1 theta dtheta2 abs(dtheta1+2*pi*(dtheta1<0))*radius lineDist*cos(ddtheta) abs(dtheta2-2*pi*(dtheta2>0))*radius];
else
    dLSR=inf;
    data(2,:)=zeros(1,6);
    data(2,5)=inf;
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

    data(3,:)=[dtheta1 theta dtheta2 abs(dtheta1-2*pi*(dtheta1>0))*radius lineDist*cos(ddtheta) abs(dtheta2+2*pi*(dtheta2<0))*radius];
else
    dRSL=inf;
    data(3,:)=zeros(1,6);
    data(3,5)=inf;
end

% 4-RSR
lineDist=norm(center1R-center2R);
if lineDist<0.00001
    theta=angle_normalize(-(alpha1-alpha2+2*pi*(alpha1-alpha2<-0.0001))*0.5+alpha1);
    dtheta1=angle_normalize(theta-alpha1);
    dtheta2=angle_normalize(alpha2-theta);
    dRSR=radius*(alpha1-alpha2+2*pi*(alpha1-alpha2<-0.0001));
    data(4,:)=[dtheta1 theta dtheta2 0.5*dRSR 0 0.5*dRSR];
else
    theta=atan2(center2R(2)-center1R(2),center2R(1)-center1R(1));
    dtheta1=angle_normalize(theta-alpha1);
    dtheta2=angle_normalize(alpha2-theta);
    cirDist=abs(dtheta1-2*pi*(dtheta1>0))*radius+abs(dtheta2-2*pi*(dtheta2>0))*radius;
    dRSR=lineDist+cirDist;
    data(4,:)=[dtheta1 theta dtheta2 abs(dtheta1-2*pi*(dtheta1>0))*radius lineDist abs(dtheta2-2*pi*(dtheta2>0))*radius];
end

[d,n]=min([dLSL, dLSR, dRSL, dRSR]);
% disp(n);

if d<=step
    A=node2.coord;
    return;
end

d=step;
dtheta1=data(n,1);
theta=data(n,2);
dtheta2=data(n,3);
c1=data(n,4);
lineDist=data(n,5);
c2=data(n,6);

if d<=c1
    eta=d/radius;
    if n==3 || n==4
        eta=-eta;
    end
    dir=alpha1+eta;
    
    if n==1 || n==2
        A(1)=center1L(1)+radius*cos(dir-pi/2);
        A(2)=center1L(2)+radius*sin(dir-pi/2);
    else
        A(1)=center1R(1)+radius*cos(dir+pi/2);
        A(2)=center1R(2)+radius*sin(dir+pi/2);
%         disp(center1R);
%         disp(cos(dir+pi/2));
%         disp(center1R(1)+radius*cos(dir+pi/2));
%         disp(A);
    end    
elseif d<=c1+lineDist
    d=d-c1;
    if n==1 || n==2
        A(1)=center1L(1)+radius*cos(theta-pi/2)+d*cos(theta);
        A(2)=center1L(2)+radius*sin(theta-pi/2)+d*sin(theta);
    else
        A(1)=center1R(1)+radius*cos(theta+pi/2)+d*cos(theta);
        A(2)=center1R(2)+radius*sin(theta+pi/2)+d*sin(theta);
    end    
else
    d=c1+lineDist+c2-d;
    eta=d/radius;
    if n==1 || n==3
        eta=-eta;
    end
    dir=alpha2+eta;
    
    if n==1 || n==3
        A(1)=center2L(1)+radius*cos(dir-pi/2);
        A(2)=center2L(2)+radius*sin(dir-pi/2);
    else
        A(1)=center2R(1)+radius*cos(dir+pi/2);
        A(2)=center2R(2)+radius*sin(dir+pi/2);
    end
end
% disp(center1L);
% disp(center2R);
% disp([d<=c1, d<=c1+lineDist])
% disp([center1L,center2R])
% disp(norm(center1L-center2R))
% disp(lineDist)
% disp([d, c1, lineDist, c2])
% disp(n)
% disp(A);
return;
end