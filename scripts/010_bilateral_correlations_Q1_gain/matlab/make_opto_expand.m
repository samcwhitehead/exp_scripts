directory_name = 'C:\Users\flyranch\panels\full_motion_probe';

pattern.x_num = 96;
pattern.y_num = 8;
pattern.num_panels = 60;
pattern.gs_val = 1;
pattern.row_compression = 0;

Pats = zeros(8, 96, pattern.x_num, pattern.y_num);

% 12 panels in a circle -> 96 columns, 5 panel high -> 40 rows
InitPat = [repmat([zeros(4,4), ones(4,4)], 1,12);repmat([ones(4,4), zeros(4,4)], 1,12)];
InitPat = repmat(InitPat, 5,1);

Pats = zeros(40, 96, pattern.x_num, pattern.y_num);

Pats(:,:,1,1) = InitPat;

for j = 2:96
    Pats(:,:,j,1) = ShiftMatrix(Pats(:,:,j-1,1), 1, 'r', 'y'); 
end

for j = 1:96
    for i = 2:8
        Pats(:,:,j,i) = ShiftMatrix(Pats(:,:,j,i-1), 1, 'd', 'y'); 
    end
end

pattern.Pats = Pats;

A =                 [1   5   9   2   6   10  3   7   11  4   8   12;
                    13  17  21  14  18  22  15  19  23  16  20  24;
                    25  29  33  26  30  34  27  31  35  28  32  36;
                    37  41  45  38  42  46  39  43  47  40  44  48;
                    49  53  57  50  54  58  51  55  59  52  56  60;];

pattern.Panel_map = fliplr(A);

pattern.BitMapIndex = process_panel_map(pattern);
pattern.data = Make_pattern_vector(pattern);

str = [directory_name '\' sprintf('Pattern_full_drift')];
save(str, 'pattern');

str = [directory_name '\' sprintf('position_function_le_50hz_500ms')];
%5000ms static, 500ms expansion 4500ms static
func = [zeros(1,5*50) round((28/25)*(0:0.5*50)) zeros(1,5*50)];
save(str, 'func');
