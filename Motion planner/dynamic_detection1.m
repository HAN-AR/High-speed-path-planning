disp('Inside the dynamic collision detection function');
load 'dynamic_detection.mat'
obs_path=unnamed; 
path=path;
path_size = size(path);
dyn_path_size = size(obs_path);
%path=path_Streetdrone;
dyn_obs_path=obs_path;
g=min(path_size, dyn_path_size);
k1=[];
for i=1:1:536
    l=sqrt((path(i,1)-dyn_obs_path(i,1))^2 + (path(i,2)-dyn_obs_path(i,2))^2)
   % k1=[k1;k];
   x=(path(i,1)-dyn_obs_path(i,1));
   y=(path(i,2)-dyn_obs_path(i,2));
   k=hypot(x,y);
   [m j]=min(k)
   k1=[k1;k];
   %k= sqrt((path(i,1)-dyn_obs_path(:,1)).^2 + (path(:,2)-dyn_obs_path(:,2)).^2)
     if m<3  
        coll_detected=true;
        x1= path(i,1)-mod(path(i,1),10);
        x2=x1+10;
        y1=path(i,2)-mod(path(i,2),10)-5;
        y2=y1+7;
        coll_time=path(i,4)
        coll_time1=dyn_obs_path(i,3)
        
       
        break
    else 
        x1=inf;
        x2=inf;
        y1=inf;
        y2=inf;
        coll_time=inf;
    end
end
x=obs_path(i,1);
y=obs_path(i,2);
lz= 2; % IN zone
wz= 4;
dt1=  atand(wz/lz);
dt2=  -atand(wz/lz);
dl= hypot(wz/2,lz/2);
x1=  x + dl*cosd(dt1);
y1=  y + dl*sind(dt1);
x2=  x + dl*cosd(dt2);
y2=  y + (dl+3)*sind(dt2);
x3=  x + dl*cosd(dt1+180);
y3=  y + (dl+3)*sind(dt1+180);
x4=  x + dl*cosd(dt2+180);
y4=  y + dl*sind(dt2+180);

obsx = [x1 x2 x3 x4]';
obsy = [y1 y2 y3 y4]';
 dyn_obst= polyshape([obsx(1,1);obsx(2,1);obsx(3,1);obsx(4,1)],[obsy(1,1);obsy(2,1);obsy(3,1);obsy(4,1)]);
  figure(1)
   subplot(211)
    plot(dyn_obst)
        ylim([-12 10])
xlim([25 105])