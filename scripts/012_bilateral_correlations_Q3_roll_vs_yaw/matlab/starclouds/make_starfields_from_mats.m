starfield_names = {'roll_right.mat','yaw_right.mat'};

for name_num = 1:2
    % make_6_wide_med_cont_pattern_48.m
    pattern.x_num = 105;
    pattern.y_num = 1;
    %[pattern.y_num,z] = size(starfield_names)
    pattern.num_panels = 60; 	% This is the number of unique Panel IDs required.
    pattern.gs_val = 3; 	% This pattern will use 8 intensity levels
    pattern.row_compression = 0;

    Pats = zeros(40, 96, pattern.x_num, pattern.y_num);
    %for j = 1:pattern.y_num
    pname = starfield_names(name_num)
    load(char(pname));
    Pats(:,:,:,:) = imgs;

    pattern.Pats = Pats;

    pattern.Panel_map = [12 8 4 11 7 3 10 6 2  9 5 1; 
                         24 20 16 23 19 15 22 18 14 21 17 13; 
                         36 32 28 35 31 27 34 30 26 33 29 25; 
                         48 44 40 47 43 39 46 42 38 45 41 37;
                         48+12 44+12 40+12 47+12 43+12 39+12 46+12 42+12 38+12 45+12 41+12 37+12];
    pattern.BitMapIndex = process_panel_map(pattern);
    pattern.data = Make_pattern_vector(pattern);


    directory_name = 'I:/starclouds/';
    fname = strcat(directory_name, 'Pattern_', starfield_names(name_num))
    fname = fname(1)
    save(char([fname(1)]), 'pattern');
end