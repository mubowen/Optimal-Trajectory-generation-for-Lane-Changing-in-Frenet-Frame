function [rx,ry ,ryaw,rk,s,objSpline]=calcSplineCourse(x, y, ds)
    objSpline=Spline2D(x,y);
    s=0:ds:objSpline.s(end);
    
    rx=[];
    ry=[];
    ryaw=[];
    rk=[];
    for i_s = s
        [ix,iy]=objSpline.calc_position(i_s);
        rx(end+1)=ix;
        ry(end+1)=iy;
        ryaw(end+1)=objSpline.calc_yaw(i_s);
        rk(end+1)=objSpline.calc_curvature(i_s);
    end
      

end 