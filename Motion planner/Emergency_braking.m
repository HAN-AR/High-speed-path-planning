function Emergency_brake = Emergency_braking(cur,obsx,obsy,obsc)
%to activate the emergency braking
L = 1.686; % Wheelbase of Streetdrone [m]
oh_r = 1;       % Longitudinal distance from the rear axle to the end of the vehicle [m] 
oh_f = 3;  % Longitudinal distance from the rear axle to the front of the vehicle [m]
w_1   =1.5;     % Width of vehicle [m]
% Now  we will create vectors to each corner of the vehicle
% Length of vector 1, 2 is the same and 3,4 is the same
lv12_1 = hypot(oh_r,(w_1/2)); % Length of Vector 1 and 2
lv34_1 = hypot((w_1/2),oh_f); % Length of Vector 3 and 4
thetap=cur.t;
cur.xa=cur.xa;
cur.ya=cur.ya;
% Angle of the vectors
av1 = (180/pi*thetap)+90+atand(oh_r/(w_1/2)); % Angle of Vector 1
av2 = (180/pi*thetap)-90-atand(oh_r/(w_1/2)); % Angle of Vector 2
av3 = (180/pi*thetap)-atand((w_1/2)/oh_f); % Angle of Vector 3
av4 = (180/pi*thetap)+atand((w_1/2)/oh_f); % Angle of Vector 4

% Finding the actual points
xv1_1 = lv12_1*cosd(av1)+cur.xa;
yv1_1 = lv12_1*sind(av1)+cur.ya;
xv2_1 = lv12_1*cosd(av2)+cur.xa;
yv2_1 = lv12_1*sind(av2)+cur.ya;
xv3_1 = lv34_1*cosd(av3)+cur.xa;
yv3_1 = lv34_1*sind(av3)+cur.ya;
xv4_1 = lv34_1*cosd(av4)+cur.xa;
yv4_1 = lv34_1*sind(av4)+cur.ya;


xc=[xv1_1;xv2_1;xv3_1;xv4_1];
yc=[yv1_1;yv2_1;yv3_1;yv4_1];



  for j=1:obsc
        obsxtemp=obsx(j,:);
        obsytemp=obsy(j,:);
        obsxtemp=obsxtemp(obsxtemp~=0);
        obsytemp=obsytemp(obsytemp~=0);
       
        in12 = inpolygon(obsxtemp,obsytemp,xc,yc); % check whether cur is hitting obs
        in12 = any(in12);
        if in12 == 1 
            Emergency_brake = true;
%              figure(3)
%              subplot(211)
%     streetdrone= polyshape([xc(1,1);xc(2,1);xc(3,1);xc(4,1)],[yc(1,1);yc(2,1);yc(3,1);yc(4,1)]);
%      plot(streetdrone)
        else
            Emergency_brake = false;
          
        end
    end


end