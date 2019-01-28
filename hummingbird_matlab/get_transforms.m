function transforms=get_transforms(tf_msgs, reference, target)
    N = length(tf_msgs);
    transforms = cell(N,1);
    count = 1;
    for i=1:N
        m = tf_msgs{i};
        if strcmp(m.Transforms.ChildFrameId, target) && ...
           strcmp(m.Transforms.Header.FrameId, reference)
            transforms{count} = m;
            count = count + 1;
        end
    end
    transforms = transforms(1:count-1);
end