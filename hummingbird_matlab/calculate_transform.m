% Given two sets of transformations from the same frame to frame1
% and frame2 respectively, this computes the transform from 
% frame1 to frame 2
% assumes transforms are ordered. drops transforms with timestamps
% that do not appear in the other set

% e.g. transforms1 is transform from world to link1, transforms2 is
% transform from world to link2, this computes transform from
% link1 to link2
function transforms=calculate_transform(transforms1, transforms2)
    N = min([length(transforms1) length(transforms2)]);
    transforms = cell(N, 1);
    i = 1; % index transforms1
    j = 1; % index transforms2
    k = 1; % indexes transforms
    while i < N && j < N
        if transforms1{i}.timestamp == transforms2{j}.timestamp
            q1 = transforms1{i}.rotation;
            q2 = transforms2{j}.rotation;
            p1 = transforms1{i}.translation;
            p2 = transforms2{j}.translation;
            timestamp = transforms1{i}.timestamp;
            transforms{k}.rotation = q1.conj*q2;
            transforms{k}.translation = q1.conj.rotatepoint(p2-p1);
            transforms{k}.timestamp = timestamp;
            k = k + 1;
            i = i + 1;
            j = j + 1;
        elseif transforms1{i}.timestamp < transforms2{j}.timestamp
            i = i + 1;
        else
            j = j + 1;
        end
    end
    transforms = transforms(1:k-1);
end