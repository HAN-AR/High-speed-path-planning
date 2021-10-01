function hc = hcost2(ip,f_state,i_state)
% This function calculates the heuristic costs in navigation maneuver
 coder.extrinsic('InPolygon')
 coder.extrinsic('reedsdistance')
%% Cost of distance from the final point

% x1=ip.xa-f_state.x;
% y1=ip.ya-f_state.y;
% t1=ip.pred.tf-f_state.t ;
% checkx= x1==round(h_xa);
% checky= y1==round(h_ya);
% checkt= t1==h_tf;
% 
% cond = all([checkx;checky;checkt;]);
% %cond=([checkt]);
% index1= find(cond);
% h_g1=h_g(:,index1);
% hg1=max(h_g1);
% ifcheck =   ~isempty(index1); 
L=1.686; % Wheel base of the semi-trailer [m]
r=L/tan(0.5235); % Radius of curvature [m]
xi=[ip.xa,ip.ya,ip.tf]; % Initial configuration
xe=[f_state.x,f_state.y,f_state.t]; % Final configuration 
[l,t,u,v,num] = reedsdistance(xi(1),xi(2),xi(3)/180*pi,xe(1),xe(2),xe(3)/180*pi,r);

%dist =  ((abs(ip.xa-f_state.x)) + (abs(ip.ya-f_state.y)))
dist =  sqrt(((ip.xa-f_state.x)^2) + ((ip.ya-f_state.y)^2)); % Euclidean distance
%dist=abs(ip.xa-f_state.x)+abs(ip.ya-f_state.y)
hc_d1= dist*1.2; %Heuristic cost times the heuristic coefficient. 


 

check = 1;lz=6; % IN zone
wz= 2;
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
in1 = InPolygon(ip.xa,ip.ya,xv,yv); % check whether cur is in the zone
xv=ppround(xv);
yv=ppround(yv);
in2 = InPolygon(ip.xa,ip.ya,xv,yv); % check whether cur is in the zone
if in1
    hc_1=l*1.1;
else
    hc_1=l*1.2;
end

  hc_d=max(hc_1,hc_d1);
lz1=6; % IN zone
wz1= 2;
dt3= i_state.t + atand(wz1/lz1);
dt4= i_state.t - atand(wz1/lz1);
d2= hypot(wz1/2,lz1/2);
x5=  i_state.x + d2*cosd(dt3);
y5=  i_state.y + d2*sind(dt3);
x6=  i_state.x + d2*cosd(dt4);
y6=  i_state.y + d2*sind(dt4);
x7=  i_state.x + d2*cosd(dt3+180);
y7=  i_state.y + d2*sind(dt3+180);
x8=  i_state.x + d2*cosd(dt4+180);
y8=  i_state.y + d2*sind(dt4+180);

xv1 = [x5 x6 x7 x8];
yv1 = [y5 y6 y7 y8];
in3 = inpolygon(ip.xa,ip.ya,xv1,yv1);
if  in3
     hc_d= hc_d*1000;
end
% If in the final zone and the theta is not the required theta then the
% cost is heavily penalized. 
if in2 && ~in1
    hc_d= hc_d*1000;
end
if in1 && f_state.t ~= ip.tf
    hc_d= hc_d*1000;
end
if in1 && abs(f_state.x-ip.x)>2
    hc_d= hc_d*1000;
end
%   if ip.pred.dir==1 && abs(i_state.t-wrapTo180(ip.tf))>=4 %|| ip.pred.tf==90 || ip.pred.tf==270 || ip.pred.tf==180)
%       hc_d= hc_d*10;
%  end

hc = hc_d;

end
