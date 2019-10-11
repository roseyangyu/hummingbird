function transforms=get_transforms(tf_msgs, reference, target)
    N = length(tf_msgs);
    transforms = cell(N,1);
    count = 1;
    for i=1:N
        messages = tf_msgs{i};
        N2 = length(messages.Transforms);
        for j=1:N2
            if strcmp(messages.Transforms(j).ChildFrameId, target) && ...
               strcmp(messages.Transforms(j).Header.FrameId, reference)
                transforms{count}.rotation = [...
                    messages.Transforms(j).Transform.Rotation.W,...
                    messages.Transforms(j).Transform.Rotation.X,...
                    messages.Transforms(j).Transform.Rotation.Y,...
                    messages.Transforms(j).Transform.Rotation.Z];
                transforms{count}.translation = [...
                    messages.Transforms(j).Transform.Translation.X,...
                    messages.Transforms(j).Transform.Translation.Y,...
                    messages.Transforms(j).Transform.Translation.Z];
                transforms{count}.timestamp = ...
                    messages.Transforms(j).Header.Stamp.Sec + ...
                    messages.Transforms(j).Header.Stamp.Nsec/1e9;
                count = count + 1;
            end
        end
    end
    transforms = transforms(1:count-1);
end