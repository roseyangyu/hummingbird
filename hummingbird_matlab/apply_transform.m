% Applies a static transform to all transformations in transforms

% Assumes transforms is a list of structs containing .rotation
% that is a matlab quaternion and .translation which is a row vector
function transforms=apply_transform(transforms, q, o)
    N = length(transforms);
    for i=1:N
        transforms{i}.rotation = transforms{i}.rotation*q;
        transforms{i}.translation = transforms{i}.translation + q.rotatepoint(o);
    end
end