function [pts, tvec, totaldist] = bezier(t, pt1, pt2, pt3, pt4)
    pts = kron((1-t).^3,pt1') + kron(3*(1-t).^2.*t,pt2') + kron(3*(1-t).*t.^2,pt3') + kron(t.^3,pt4');

    a = -3*t.^2 +  6*t - 3;
    b =  9*t.^2 - 12*t + 3;
    c = -9*t.^2 +  6*t;
    d =  3*t.^2;

    tvec = kron(a,pt1') + kron(b,pt2') + kron(c,pt3') + kron(d,pt4');
    
    totaldist = 0;
    for i = 1:length(pts)-1
       totaldist = totaldist + norm(pts(:,i+1) - pts(:,i));
    end

    pts = pts';
    tvec = tvec';
end