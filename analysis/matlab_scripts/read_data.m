% foundL = xlsread([root str '\log2.csv'], 'AF:AF');
% [num,txt,velX] = xlsread([root str '\log2.csv'], 'AM:AM');
% [num,txt,posX] = xlsread([root str '\log2.csv'], 'AG:AG');
% [num,txt,posZ] = xlsread([root str '\log2.csv'], 'AI:AI');

if insect_nr == 58
    
end

if exist([root str '\log' num2str(insect_nr) '.csv'])
    if folder_nr <600
        [foundL,posX,posY,posZ,velX,velY,velZ] = importDataOld([root str '\log' num2str(insect_nr) '.csv']);
    else
        [foundL,posX,posY,posZ,velX,velY,velZ] = importData([root str '\log' num2str(insect_nr) '.csv']);
    end

else
    foundL = [];
end    