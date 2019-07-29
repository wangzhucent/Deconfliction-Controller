function outputSys=activateCenter(Sys,q)
ActivatedCenters=Sys.ActivatedCenters;
m=size(ActivatedCenters,1);
for i=1:m
    if norm(ActivatedCenters(i,1:2)-q(1:2))<5 
        ActivatedCenters(i,3)=ActivatedCenters(i,3)+1;
        Sys.ActivatedCenters=ActivatedCenters;
        outputSys=Sys;
        return;        
    end
end

ActivatedCenters=[ActivatedCenters; q(1:2) 1];
Sys.ActivatedCenters=ActivatedCenters;
outputSys=Sys;

end