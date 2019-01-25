for i = 1:length(imageStructs)
    img = readImage(imageStructs{i});
    img_seq(:,:,:,i) = img;
end
implay(img_seq)