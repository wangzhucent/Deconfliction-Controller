function newpath=fixPath(path, rho, step)
    i=1;
    addnum=0;
    while i~=size(path,1)
%         size(path)
        q1.coord=path(i,1:2);
        q1.dir=path(i,3);
        q2.coord=path(i+1,1:2);
        q2.dir=path(i+1,3);
        tempdist=distDubinThree(q1,q2,rho);
        if tempdist<step*1.5 || (norm(q1.coord-q2.coord)<step)
            i=i+1;
        else
            temp=[steer_DubinThree(q1,q2,rho,step) steer_DubinThree2(q1,q2,rho,step)];
            path=[path(1:i,:); temp; path(i+1:end,:)];
            i=i+1;
%             addnum=addnum+1
        end
%         if i>100
%             break;
%         end
    end
    newpath=path;
end
