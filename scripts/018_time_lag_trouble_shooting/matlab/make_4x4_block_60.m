%Make_4x4_blocks_12Panels

% 12 panel pattern
pattern.x_num = 96;
pattern.y_num = 8;
pattern.num_panels = 60;
pattern.gs_val = 1;
pattern.row_compression = 0;

% 12 panels in a circle -> 96 columns, 4 panel high -> 32 rows
InitPat = [repmat([zeros(4,4), ones(4,4)], 1,12);repmat([ones(4,4), zeros(4,4)], 1,12)];
InitPat = repmat(InitPat, 5,1);

Pats = zeros(40, 96, pattern.x_num, pattern.y_num);

Pats(:,:,1,1) = InitPat;

for i = 2:96
    Pats(:,:,i,1) = ShiftMatrix(Pats(:,:,i-1,1), 1, 'r', 'y'); 
end

for j = 2:8
    for i = 1:96
        Pats(:,:,i,j) = ShiftMatrix(Pats(:,:,i,j-1), 1, 'd', 'y'); 
    end
end


pattern.Pats = Pats;

% A = 1:48;
% pattern.Panel_map = flipud(reshape(A, 4, 12));
% %     4     8    12    16    20    24    28    32    36    40    44    48
% %     3     7    11    15    19    23    27    31    35    39    43    47
% %     2     6    10    14    18    22    26    30    34    38    42    46
% %     1     5     9    13    17    21    25    29    33    37    41    45
pattern.Panel_map = [1   5   9   2   6   10  3   7   11  4   8   12;
                    13  17  21  14  18  22  15  19  23  16  20  24;
                    25  29  33  26  30  34  27  31  35  28  32  36;
                    37  41  45  38  42  46  39  43  47  40  44  48;
                    49  53  57  50  54  58  51  55  59  52  56  60;]

%pattern.Panel_map = [12  8  4 11  7  3 10  6  2  9  5  1; 
%                     24 20 16 23 19 15 22 18 14 21 17 13; 
%                     36 32 28 35 31 27 34 30 26 33 29 25; 
%                     48 44 40 47 43 39 46 42 38 45 41 37;
%                      8 44 40 47 43 39 46 42 38 45 41 37];

pattern.BitMapIndex = process_panel_map(pattern);
pattern.data = Make_pattern_vector(pattern);
directory_name = 'C:\Users\flyranch\panels\manafold_test\patterns';
%directory_name = 'c:\matlabroot\Panels\Patterns';
str = [directory_name '\Pattern_4x4_blocks_60']
save(str, 'pattern');