
str = num2str(folder_nr);

if folder_nr<10
    str = ['0000' str];
elseif folder_nr<100
    str = ['000' str];
elseif folder_nr<1000
    str = ['00' str];
elseif folder_nr<10000
    str = ['0' str];
end

fileID = fopen([root str '\insect.log'],'r');

number_of_moths = 0;
number_of_detections = 0;
moth_list = 0;
insect_nr = 0;

if fileID > 0

    tline = fgetl(fileID);
    
    if ischar(tline)

        end_id = str2num(tline(85:end));
        time = datevec(tline(25:43));

        while ischar(tline)
            
%             if time(4) > 12
                read_data;
%             end

            end_id = str2num(tline(85:end));
            time = datevec(tline(25:43));

            number_of_detections = number_of_detections+1;
%             insect_nr
            filter_insects_from_logs;

            tline = fgetl(fileID);
            insect_nr = insect_nr+1;


        end
        


    end
    
    fclose(fileID);
    
end

detections_vs_moths = [num2str(number_of_detections) ' ' num2str(number_of_moths) ' video numbers: ' num2str(moth_list(2:end))]






