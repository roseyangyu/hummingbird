for i = 1:length(imageStructs)
    img = readImage(imageStructs{i});
    img_gray = rgb2gray(img);
    tag_info = apriltags(img_gray);
end