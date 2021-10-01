function hc = hcost(ip,f_state,obs_range,delta_diff,cost1)
    coder.extrinsic('InPolygon') 
% This function calculates the heuristic costs for navigation Maneuver
%% Cost of distance from the final point

dist =  sqrt(((ip.xa-f_state.x)^2) + ((ip.ya-f_state.y)^2)); % Euclidean distance
hc_d1= dist; %Heuristic cost times the heuristic coefficient. 

if  dist<8 
      hc_d=hc_d1*1.1;

  else
    hc_d=(hc_d1)*1.2;
end

%check = 1;
lz= 10; % IN zone
wz= 4;
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
xv=round(xv);
yv=round(yv);
in2 = InPolygon(ip.xa,ip.ya,xv,yv); % check whether cur is in the zone


% If in the final zone and the theta is not the required theta then the
% cost is heavily penalized. 
if in2 && ~in1
    hc_d= hc_d*1000;
end
 if in2 && abs(ip.ya-f_state.y)>=0.3
     hc_d= hc_d*1000;  
  end
if in1 && (f_state.t )~= ( ip.tf)
  
    hc_d= hc_d*1000;
end
     if dist>1 && obs_range==1 && abs(ip.ya-f_state.y)>=0.5
         hc_d=hc_d*(1+(abs(ip.y-f_state.y)));
     end
   if dist>7  && ip.tf~=0
        hc_d=hc_d*10;
   end
%     if obs_range==1  && maxdelta>=0.2
%        hc_d=hc_d*100;
%    end
%       if obs_range==0 && abs(f_state.y-ip.y)<1.6
%           hc_d=hc_d*100;
%       end

  if dist>1 && abs(f_state.y-ip.y)>=3.1
      hc_d=hc_d*100;
  end
  if obs_range==0 && cost1>12.5
          hc_d=hc_d*100;
  end  
if dist<2 && cost1>4
      hc_d=hc_d*100;
end
%    if dist<10 && obs_range==1
%        hc_d=hc_d+(delta_diff*100);
%    end
hc = hc_d;
 end

