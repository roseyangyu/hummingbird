% Given two sets of transformations from the same frame to frame1
% and frame2 respectively, this computes the transform from 
% frame1 to frame 2

% e.g. transforms1 is transform from world to link1, transforms2 is
% transform from world to link2, this computes transform from
% link1 to link2
function transforms=calculate_transform(transforms1, transforms2)
    N = length(transforms1);
    transforms = cell(N, 1);
    for i=1:N
        q1 = transforms1{i}.rotation;
        q2 = transforms2{i}.rotation;
        p1 = transforms1{i}.translation;
        p2 = transforms2{i}.translation;
        transforms{i}.rotation = q1.conj*q2;
        transforms{i}.translation = q1.conj.rotatepoint(p2-p1);
        
    end
end