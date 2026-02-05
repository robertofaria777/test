filename = 'C:\Users\fyrf2\Downloads\monza-graphic.jpeg';
rgb = imread(filename);
Red = rgb(:,:,1); Green = rgb(:,:,2); Blue = rgb(:,:,3);
bw = Blue > 150 & Red < 128;
imshow(rgb)

B = bwboundaries(bw);
plot(B{1}(:,2), B{1}(:,1), B{2}(:,2), B{2}(:,1), B{3}(:,2), B{3}(:,1))
set(gca, 'YDir', 'reverse')
