function  check = DynObs_check(cur,xp,yp,thetap,dobsx,dobsy)
  coder.extrinsic('inpolygon')
  %check the collison with dynamic obstacle 
check=1;     % Collision check variable
% global obsc;
% We have the position of the vehicle axle(xp,yp) and the orientation of the
% vehicle(thetap). So from all this
% information we can find all the four corners of the trailer and the four
% corners of the tractor and see if these corners are hitting the obstacle.
%%
L = 1.686; % Wheelbase of Streetdrone [m]
oh_r = 0.2;       % Longitudinal distance from the rear axle to the end of the vehicle [m] 
oh_f = L+0.3;  % Longitudinal distance from the rear axle to the front of the vehicle [m]
w_1   = 1.5;     % Width of vehicle [m]
% Now  we will create vectors to each corner of the vehicle
% Length of vector 1, 2 is the same and 3,4 is the same
lv12_1 = hypot(oh_r,(w_1/2)); % Length of Vector 1 and 2
lv34_1 = hypot((w_1/2),oh_f); % Length of Vector 3 and 4
for i=1:1:length(xp)
    xp1=xp(i,1);
    yp1=yp(i,1);
    thetap1=thetap(i,1);
% Angle of the vectors
av1 = (180/pi*thetap1)+90+atand(oh_r/(w_1/2)); % Angle of Vector 1
av2 = (180/pi*thetap1)-90-atand(oh_r/(w_1/2)); % Angle of Vector 2
av3 = (180/pi*thetap1)-atand((w_1/2)/oh_f); % Angle of Vector 3
av4 = (180/pi*thetap1)+atand((w_1/2)/oh_f); % Angle of Vector 4

% Finding the actual points
xv1_1 = xp1+lv12_1*cosd(av1)+cur.xa;
yv1_1 = yp1+lv12_1*sind(av1)+cur.ya;
xv2_1 = xp1+lv12_1*cosd(av2)+cur.xa;
yv2_1 = yp1+lv12_1*sind(av2)+cur.ya;
xv3_1 = xp1+lv34_1*cosd(av3)+cur.xa;
yv3_1 = yp1+lv34_1*sind(av3)+cur.ya;
xv4_1 = xp1+lv34_1*cosd(av4)+cur.xa;
yv4_1 = yp1+lv34_1*sind(av4)+cur.ya;


xc=[xv1_1 xv2_1 xv3_1 xv4_1 xv1_1];
yc=[yv1_1 yv2_1 yv3_1 yv4_1 yv1_1];

cibxmax= any(xc>=160);
cibymax= any(yc>10);
cibxmin= any(xc<=-10);
cibymin= any(yc<=-10);

%     
%         obsxtemp=obsx;
%         obsytemp=obsy;
%         obsxtemp=obsxtemp(obsxtemp~=0);
%         obsytemp=obsytemp(obsytemp~=0);
        in12=zeros(length(dobsx),1, 'logical');
        in12 = inpolygon(dobsx,dobsy,xc,yc); 
        in1 = any(in12);
        if in1==1 ||cibxmax==1||cibymax==1||cibxmin==1||cibymin==1
         check = 0;
         return;
        end
    


end