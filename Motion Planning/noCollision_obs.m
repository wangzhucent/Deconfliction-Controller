function nc = noCollision_obs(n2, n1, ob_vec)
 nc=1;
 [m,n]=size(ob_vec);
 for i=1:m
     nc=nc*noCollision(n2,n1,ob_vec(i,:));
 end
end