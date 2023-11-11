root = '/home/houjebek/Bla/pats/monitoring/koppert/4/data_logs';

  
folders = [1:766];
nr_of_folders = length(folders);
time_list = zeros(1,6);

for ii = 1:nr_of_folders
    
    folder_nr = folders(ii)
    read_insect_txt;

end    
    
time_list = time_list(2:end,:);


