
% In this function we check if the vehicle has reached the final goal

function check = loop_check(cur,f_state)
  coder.extrinsic('InPolygon') 
%Instead of checking if the final point described by the f_sstate we will 
%if the path planner has reached a zone enclosed around the final state. 
% Check the orientation of the final state and make the zone accordingly
% Creating zone
 check = 1;
lz=1; % IN zone
wz= 1;
dt1= f_state.t + atand(wz/lz);
dt2= f_state.t - atand(wz/lz);
dl= hypot(wz/2,lz/2);
x1=  f_state.x + dl*cosd(dt1);
y1=  f_state.y + dl*sind(dt1);
x2=  f_state.x + dl*cosd(dt2);
y2=  f_state.y + dl*sind(dt2);
x3=  f_state.x + dl*cosd(dt1+180);
y3=  f_state.y + dl*sind(dt1+180);
x4=  f_state.x + dl*cosd(dt2+180);
y4=  f_state.y + dl*sind(dt2+180);
xv = [x1 x2 x3 x4];
yv = [y1 y2 y3 y4];
 in = InPolygon(cur.xa,cur.ya,xv,yv); % check whether cur is in the zone
% % %Check weather the final conditions are reached.
      if in && cur.tf == f_state.t 
   
   % if  (cur.x==f_state.x)  && (cur.y == f_state.y) && (cur.tf == f_state.t)
      
        check = 0;  % this will jump out of the while loop in Path_planner.m
      end
     
  
end