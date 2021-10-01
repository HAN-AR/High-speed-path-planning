function [x1,y1,coll_time,idx]=dynamic_collison(path,obs_path,thetad)
disp('Inside the dynamic collision detection function');

x1=0;
y1=0;
coll_time=0;
idx=0;
path_size = size(path(:,1));
dyn_path_size = size(obs_path(:,1));
%path=path_Streetdrone;
dyn_obs_path=obs_path;
g=min(path_size, dyn_path_size);
%k1=[];
for i=1:1:g(1)
   
    l=sqrt((path(i,1)-dyn_obs_path(i,1))^2 + (path(i,2)-dyn_obs_path(i,2))^2);
   % k1=[k1;k];
   x=(path(i,1)-dyn_obs_path(i,1));
   y=(path(i,2)-dyn_obs_path(i,2));
   k=hypot(x,y);
  % [m j]=min(k);
 %  k1=[k1;k];
   %k= sqrt((path(i,1)-dyn_obs_path(:,1)).^2 + (path(:,2)-dyn_obs_path(:,2)).^2)
   %%
   xp=path(i,1);
yp=path(i,2);
thetap=path(i,3);
   L_1f = 1.685; % Wheelbase of streetdrone 
    oh_1b = 0.3;       % Longitudinal distance from the rear axle to the end of the vehicle [m] 
    oh_1f = L_1f+0.3;  % Longitudinal distance from the rear axle to the front of the vehicle [m]
    w_1   = 1.2;     % Width of streetdrone [m]
    % Now  we will create vectors to each corner of the vehicle
    % Length of vector 1, 2 is the same and 3,4 is the same
    lv12_1 = hypot(oh_1b,(w_1/2)); % Length of Vector 1 and 2
    lv34_1 = hypot((w_1/2),oh_1f); % Length of Vector 3 and 4
 av1 = (180/pi*thetap)+90+atand(oh_1b/(w_1/2)); % Angle of Vector 1
    av2 = (180/pi*thetap)-90-atand(oh_1b/(w_1/2)); % Angle of Vector 2
    av3 = (180/pi*thetap)-atand((w_1/2)/oh_1f); % Angle of Vector 3
    av4 = (180/pi*thetap)+atand((w_1/2)/oh_1f); % Angle of Vector 4

% Finding the actual points
 xv1_1 = xp+lv12_1*cosd(av1);
    yv1_1 = yp+lv12_1*sind(av1);
    xv2_1 = xp+lv12_1*cosd(av2);
    yv2_1 = yp+lv12_1*sind(av2);
    xv3_1 = xp+lv34_1*cosd(av3);
    yv3_1 = yp+lv34_1*sind(av3);
    xv4_1 = xp+lv34_1*cosd(av4);
    yv4_1 = yp+lv34_1*sind(av4);

 xc=[[xv1_1 xv2_1 xv3_1 xv4_1]]';
    yc=[[yv1_1 yv2_1 yv3_1 yv4_1]]';
   %streetdrone= polyshape([xc(1,1);xc(2,1);xc(3,1);xc(4,1)],[yc(1,1);yc(2,1);yc(3,1);yc(4,1)]);
   x=obs_path(i,1);
y=obs_path(i,2);
lz= 2; % IN zone
wz= 1.5;
 dt1 = (180/pi*thetad)+90+atand(wz/lz); % Angle of Vector 1
 dt2 = (180/pi*thetad)-90-atand(wz/lz); % Angle of Vector 2
 dt3 = (180/pi*thetad)-atand(wz/lz); % Angle of Vector 3
 dt4 = (180/pi*thetad)+atand(wz/lz); % Angle of Vector 4
%dt1=  atand(wz/lz);
%dt2=  -atand(wz/lz);
dl= hypot(wz/2,lz/2);
dx1=  x + dl*cosd(dt1);
dy1=  y + dl*sind(dt1);
dx2=  x + dl*cosd(dt2);
dy2=  y + (dl)*sind(dt2);
dx3=  x + dl*cosd(dt3);
dy3=  y + (dl)*sind(dt3);
dx4=  x + dl*cosd(dt4);
dy4=  y + dl*sind(dt4);

obsx = [dx1 dx2 dx3 dx4]';
obsy = [dy1 dy2 dy3 dy4]';
% dyn_obst= polyshape([obsx(1,1);obsx(2,1);obsx(3,1);obsx(4,1)],[obsy(1,1);obsy(2,1);obsy(3,1);obsy(4,1)]);
  Intersect=inpolygon(obsx,obsy,xc,yc);
  Intersect1=inpolygon(xc,yc,obsx,obsy);
  Intersect=any(Intersect);
  Intersect1=any(Intersect1);
   if Intersect==1 || Intersect1==1
        coll_detected=true;
        coll_time=path(i,4);
        coll_time1=dyn_obs_path(i,3);
        stop_time=coll_time-4;
        [min_dist,idx]=min(abs(path(:,4)-stop_time));
        x1= path(idx,1);
        x2=x1+10;
        y1=path(idx,2);
        y2=y1+7;
        
       %  figure(1)
  % subplot(211)
%     plot(dyn_obst)
%     hold on
%     axis equal
%     plot(streetdrone,'FaceColor','y','FaceAlpha',0.01)  
        break
    else 
        x1=inf;
        x2=inf;
        y1=inf;
        y2=inf;
        coll_time=inf;
        idx=inf;
    end
end

end