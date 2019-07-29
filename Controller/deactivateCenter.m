function outputSys=deactivateCenter(Sys,q)
ActivatedCenters=Sys.ActivatedCenters;
m=size(ActivatedCenters,1);
for i=1:m
    if norm(ActivatedCenters(i,1:2)-q(1:2))<5 
        ActivatedCenters(i,3)=ActivatedCenters(i,3)-1;
        if ActivatedCenters(i,3)<=0
            ActivatedCenters=ActivatedCenters([1:i-1, i+1:end],:);
        end
        Sys.ActivatedCenters=ActivatedCenters;
        outputSys=Sys;
        return;        
    end
end

disp('Error: no such activated center found in system!');

end