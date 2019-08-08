function s=cal_s(x,y)

dx=diff(x);
dy=diff(y);
ds=sqrt(dx*dx'+dy*dy');
s=[0, cusum(s)];


end