
% find detection occurences

f = foundL;

% detections = find((f(2:end_id)-f(1:end_id-1))>0,1,'last');
% detections_end = find((f(detections+1:end_id)-f(detections:end_id-1))<0);

vel_temp_list = zeros(1,1);
posX_list = zeros(1,1);
posZ_list = zeros(1,1);

vel = 0;
for j = 2:length(foundL)
    
%     velX(j,1)
 if abs(velX(j,1)-velX(j-1,1))>0.01 || abs(velZ(j,1)-velZ(j-1,1))>0.01
        vel = vel+1;
        if abs(velX(j,1))>0
        vel_temp_list = [vel_temp_list velX(j,1)];
        end
        
        if abs(posX(j,1))>0
            posX_list = [posX_list (posX(j,1))];
        end
        
        if abs(posZ(j,1)) > 0
            posZ_list = [posZ_list (posZ(j,1))];
        end
 end
        
%         posZ_list = [posZ_list (posZ(j,1))];
        
end

% vel_list = [vel_list; vel];
posX_list = posX_list(2:end);
posZ_list = posZ_list(2:end);

% figure(1)
% plot(posX_list(2:end))

max_diffX = max(posX_list)-min(posX_list);
max_diffZ = max(posZ_list)-min(posZ_list);

% if insect_nr == 168
if length(posX_list)>0 &&((max_diffX > 0.050) || (max_diffZ > 0.050)) && vel>sum(foundL==1)/3
    number_of_moths = number_of_moths+1;
    moth_list = [moth_list insect_nr];
    time_list = [time_list; time];
end
% end