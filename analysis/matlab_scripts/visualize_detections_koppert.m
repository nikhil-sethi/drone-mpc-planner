day_count_sept4 = zeros(30,1);
day_count_okt4 = zeros(31,1);
day_count_nov4 = zeros(30,1);

day_count_sept6 = zeros(30,1);
day_count_okt6 = zeros(31,1);
day_count_nov6 = zeros(30,1);


for i = 1:size(time_list_complete6,1)
    
    % check time (after half past 8)
    if ((time_list_complete6(i,4)==19 ...
            && time_list_complete6(i,5) > 30)...
            || time_list_complete6(i,4)>19)...
            
        
        %bin per day
        if time_list_complete6(i,2) == 9
            day_count_sept6(time_list_complete6(i,3)) = day_count_sept6(time_list_complete6(i,3))+1;
        end
        if time_list_complete6(i,2) == 10
            day_count_okt6(time_list_complete6(i,3)) = day_count_okt6(time_list_complete6(i,3))+1;
        end
        if time_list_complete6(i,2) == 11
            day_count_nov6(time_list_complete6(i,3)) = day_count_nov6(time_list_complete6(i,3))+1;
        end
        
    end
    
end

for i = 1:size(time_list_complete4,1)
    
    % check time (after half past 8)
    if ((time_list_complete4(i,4)==19 ...
            && time_list_complete4(i,5) > 30)...
            || time_list_complete4(i,4)>19)...
            
        
        %bin per day
        if time_list_complete4(i,2) == 9
            day_count_sept4(time_list_complete4(i,3)) = day_count_sept4(time_list_complete4(i,3))+1;
        end
        if time_list_complete4(i,2) == 10
            day_count_okt4(time_list_complete4(i,3)) = day_count_okt4(time_list_complete4(i,3))+1;
        end
        if time_list_complete4(i,2) == 11
            day_count_nov4(time_list_complete4(i,3)) = day_count_nov4(time_list_complete4(i,3))+1;
        end
        
    end
    
end

day_count_okt4([ 1 7 8 9 11 15 17 22 24]) = 0;
day_count_okt4_wrong = zeros(31,1);
day_count_okt4_wrong([ 1 7 8 9 11 15 17 22 24]) = -2;

close all

ymax = 80;

figure(1)
subplot(3,3,1)
% plot(time_axis,moths_counts)
gray_factor = 0.9;
% bar(time_axis,lights,'FaceColor',[gray_factor gray_factor gray_factor],'EdgeColor',[gray_factor gray_factor gray_factor])
hold on
% axes('Xlim', [12, 30], 'XTick', 12:2:30, 'NextPlot', 'add');
bar(1:30,day_count_sept4)
title('september - systeem 1')
xlabel('datum')
ylabel('mot-detecties per avond')
xlim([12 30.5])
ylim([0 ymax])
xticks(12:2:30);

hold off

% figure(2)
subplot(3,3,2)

% plot(time_axis,moths_counts)
gray_factor = 0.9;
% bar(time_axis,lights,'FaceColor',[gray_factor gray_factor gray_factor],'EdgeColor',[gray_factor gray_factor gray_factor])
hold on
bar(1:31,day_count_okt4)
bar(1:31,day_count_okt4_wrong,'FaceColor',[gray_factor gray_factor gray_factor])
legend('correcte metingen','foutieve metingen','Location','North')

title('oktober - systeem 1')
xlabel('datum')
ylabel('mot-detecties per avond')
xlim([0.5 31.5])
ylim([-2 ymax])

xticks(1:2:31);

hold off

subplot(3,3,3)
% plot(time_axis,moths_counts)
gray_factor = 0.9;
% bar(time_axis,lights,'FaceColor',[gray_factor gray_factor gray_factor],'EdgeColor',[gray_factor gray_factor gray_factor])
hold on
% axes('Xlim', [12, 30], 'XTick', 12:2:30, 'NextPlot', 'add');
bar(1:30,day_count_nov4)
title('november - systeem 1')
xlabel('datum')
ylabel('mot-detecties per avond')
xlim([1 25.5])
ylim([0 ymax])
xticks(1:2:25);

hold off

% figure(3)
subplot(3,3,4)

% plot(time_axis,moths_counts)
gray_factor = 0.9;
% bar(time_axis,lights,'FaceColor',[gray_factor gray_factor gray_factor],'EdgeColor',[gray_factor gray_factor gray_factor])
hold on
bar(1:30,day_count_sept6)
title('september - systeem 2')
xlabel('datum')
ylabel('mot-detecties per avond')
xlim([12 30.5])
ylim([0 ymax])

xticks(12:2:30);

hold off

% figure(4)
subplot(3,3,5)

% plot(time_axis,moths_counts)
gray_factor = 0.9;
% bar(time_axis,lights,'FaceColor',[gray_factor gray_factor gray_factor],'EdgeColor',[gray_factor gray_factor gray_factor])
hold on
bar(1:31,day_count_okt6)
title('oktober - systeem 2')
xlabel('datum')
ylabel('mot-detecties per avond')
xlim([0.5 31.5])
ylim([0 ymax])

xticks(1:2:31);

hold off

subplot(3,3,6)
% plot(time_axis,moths_counts)
gray_factor = 0.9;
% bar(time_axis,lights,'FaceColor',[gray_factor gray_factor gray_factor],'EdgeColor',[gray_factor gray_factor gray_factor])
hold on
% axes('Xlim', [12, 30], 'XTick', 12:2:30, 'NextPlot', 'add');
bar(1:30,day_count_nov6)
title('november - systeem 2')
xlabel('datum')
ylabel('mot-detecties per avond')
xlim([1 25.5])
ylim([0 ymax])
xticks(1:2:25);

hold off

% figure(5)
subplot(3,3,7)

% plot(time_axis,moths_counts)
gray_factor = 0.9;
% bar(time_axis,lights,'FaceColor',[gray_factor gray_factor gray_factor],'EdgeColor',[gray_factor gray_factor gray_factor])
hold on
bar(1:30,day_count_sept4+day_count_sept6)
title('september - systeem 1+2')
xlabel('datum')
ylabel('mot-detecties per avond')
xlim([12 30.5])
ylim([0 ymax])

hold off

% figure(6)
subplot(3,3,8)

% plot(time_axis,moths_counts)
gray_factor = 0.9;
% bar(time_axis,lights,'FaceColor',[gray_factor gray_factor gray_factor],'EdgeColor',[gray_factor gray_factor gray_factor])
hold on
bar(1:31,day_count_okt4+day_count_okt6)
title('oktober - systeem 1+2')
xlabel('datum')
ylabel('mot-detecties per avond')
xlim([0.5 31.5])
ylim([0 ymax])
xticks(1:2:31);


hold off

subplot(3,3,9)
% plot(time_axis,moths_counts)
gray_factor = 0.9;
% bar(time_axis,lights,'FaceColor',[gray_factor gray_factor gray_factor],'EdgeColor',[gray_factor gray_factor gray_factor])
hold on
% axes('Xlim', [12, 30], 'XTick', 12:2:30, 'NextPlot', 'add');
bar(1:30,day_count_nov4+day_count_nov6)
title('november - systeem 1+2')
xlabel('datum')
ylabel('mot-detecties per avond')
xlim([1 25.5])
ylim([0 ymax])
xticks(1:2:25);

hold off