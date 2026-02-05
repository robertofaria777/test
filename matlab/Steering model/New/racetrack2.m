filename = 'C:\Users\User\Desktop\APP Development\test\matlab\Steering model\New\monza-graphic.jpeg';
rgb = imread(filename);

Red   = rgb(:,:,1);
Green = rgb(:,:,2);
Blue  = rgb(:,:,3);

bw = Blue > 150 & Red < 128;

B = bwboundaries(bw);

% Select only one boundary (e.g., the first)
boundary = B{1};

% Extract X = columns, Y = rows
x = boundary(:,2);
y = boundary(:,1);

% Display values
disp([x y])

% Plot only that line (optional)
figure; plot(x, y); axis equal; set(gca, 'YDir', 'reverse')
title('Single Boundary Coordinates')
