pt1 = [ 5;-10];
pt2 = [45; 15];

placelabel(pt1,'pt_1');
placelabel(pt2,'pt_2');
xlim([0 50])
axis equal

t = linspace(0,1,101);
pts = kron((1-t),pt1) + kron(t,pt2);

hold on
plot(pts(1,:),pts(2,:))
hold off

%% 3 Points
pt1 = [ 5;-10];
pt2 = [18; 18];
pt3 = [45; 15];

cla
placelabel(pt1,'pt_1');
placelabel(pt2,'pt_2');
placelabel(pt3,'pt_3');
xlim([0 50])
axis equal

pts = kron((1-t).^2,pt1) + kron(2*(1-t).*t,pt2) + kron(t.^2,pt3);

hold on
plot(pts(1,:),pts(2,:))
hold off

%% 4 Points

pt1 = [ 5;-10];
pt2 = [18; 18];
pt3 = [38; -5];
pt4 = [45; 15];

cla
placelabel(pt1,'pt_1');
placelabel(pt2,'pt_2');
placelabel(pt3,'pt_3');
placelabel(pt4,'pt_4');
xlim([0 50])
axis equal

pts = kron((1-t).^3,pt1) + kron(3*(1-t).^2.*t,pt2) + kron(3*(1-t).*t.^2,pt3) + kron(t.^3,pt4);

hold on
plot(pts(1,:),pts(2,:))
hold off

a = -3*t.^2 +  6*t - 3;
b =  9*t.^2 - 12*t + 3;
c = -9*t.^2 +  6*t;
d =  3*t.^2;

tvec = kron(a,pt1) + kron(b,pt2) + kron(c,pt3) + kron(d,pt4);

for i=1:10:101
    l = line([pts(1,i), pts(1,i)+tvec(1,i)/6], ...
             [pts(2,i), pts(2,i)+tvec(2,i)/6]);
    l.Color = 'green';
end

%% 4 Points using bezier.m

pt1 = [ 5;-10];
pt2 = [18; 18];
pt3 = [38; -5];
pt4 = [45; 15];

t = linspace(0,1,101);

cla
placelabel(pt1,'pt_1');
placelabel(pt2,'pt_2');
placelabel(pt3,'pt_3');
placelabel(pt4,'pt_4');
xlim([0 50])
axis equal

[pts, tvec, totaldist] = bezier(t, pt1, pt2, pt3, pt4);

hold on
plot(pts(1,:),pts(2,:))
hold off

for i=1:10:101
    l = line([pts(1,i), pts(1,i)+tvec(1,i)/6], ...
             [pts(2,i), pts(2,i)+tvec(2,i)/6]);
    l.Color = 'green';
end