% Applies a static transform to all transformations in transforms.
% recall: o02 = o01 + R01 o12
%         R02 = R01 R12
% where q = R12 and o = o12.

% Assumes transforms is a list of structs containing .rotation
% that is a matlab quaternion and .translation which is a row vector
function transforms=apply_transform(transforms, q, o)
    N = length(transforms);
    for i=1:N
        transforms{i}.translation = transforms{i}.translation + ...
                                    transforms{i}.rotation.rotatepoint(o);
        transforms{i}.rotation = transforms{i}.rotation*q;
    end
end