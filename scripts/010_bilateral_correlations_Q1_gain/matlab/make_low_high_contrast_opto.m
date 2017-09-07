STRIPE_WIDTH = 8;
directory_name = 'C:\Users\flyranch\panels\billateral_correlations\patterns';

for repnum = 1:3
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %Make Wide Field
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    pattern.x_num = 96;
    pattern.y_num = 1;
    pattern.num_panels = 60;
    pattern.gs_val = 3;
    pattern.row_compression = 1;

    RandPat = repmat((rand(1,96)>0.5),5,1);

    RandPats = zeros(5, 96, pattern.x_num, pattern.y_num);
    RandPats(:,:,1,1) = RandPat;
    
    for j = 2:96
      RandPats(:,:,j,1) = ShiftMatrix(RandPats(:,:,j-1,1), 1, 'r', 'y'); 
    end
    
    %Low Contrast
    Pats = RandPats*1 + 3;

    pattern.Pats = Pats;

    A =                 [1   5   9   2   6   10  3   7   11  4   8   12;
                        13  17  21  14  18  22  15  19  23  16  20  24;
                        25  29  33  26  30  34  27  31  35  28  32  36;
                        37  41  45  38  42  46  39  43  47  40  44  48;
                        49  53  57  50  54  58  51  55  59  52  56  60;];

    pattern.Panel_map = fliplr(A);

    pattern.BitMapIndex = process_panel_map(pattern);
    pattern.data = Make_pattern_vector(pattern);
    
    str = [directory_name '\' sprintf('Pattern_full_%d_LC',repnum)];
    save(str, 'pattern');
    
    %High contrast
    Pats = RandPats*7;

    pattern.Pats = Pats;

    A =                 [1   5   9   2   6   10  3   7   11  4   8   12;
                        13  17  21  14  18  22  15  19  23  16  20  24;
                        25  29  33  26  30  34  27  31  35  28  32  36;
                        37  41  45  38  42  46  39  43  47  40  44  48;
                        49  53  57  50  54  58  51  55  59  52  56  60;];

    pattern.Panel_map = fliplr(A);

    pattern.BitMapIndex = process_panel_map(pattern);
    pattern.data = Make_pattern_vector(pattern);
    
    str = [directory_name '\' sprintf('Pattern_full_%d_HC',repnum)];
    save(str, 'pattern');
    
    %1 bit HC
    pattern.x_num = 96;
    pattern.y_num = 1;
    pattern.num_panels = 60;
    pattern.gs_val = 1;
    pattern.row_compression = 1;

    RandPat = repmat((rand(1,96)>0.5),5,1);

    RandPats = zeros(5, 96, pattern.x_num, pattern.y_num);
    RandPats(:,:,1,1) = RandPat;
    
    
    Pats = RandPats;

    pattern.Pats = Pats;

    A =                 [1   5   9   2   6   10  3   7   11  4   8   12;
                        13  17  21  14  18  22  15  19  23  16  20  24;
                        25  29  33  26  30  34  27  31  35  28  32  36;
                        37  41  45  38  42  46  39  43  47  40  44  48;
                        49  53  57  50  54  58  51  55  59  52  56  60;];

    pattern.Panel_map = fliplr(A);

    pattern.BitMapIndex = process_panel_map(pattern);
    pattern.data = Make_pattern_vector(pattern);
    
    str = [directory_name '\' sprintf('Pattern_full_%d_Fast',repnum)];
    save(str, 'pattern');
end