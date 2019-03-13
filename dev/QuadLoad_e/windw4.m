function [f,df]=windw4(x,u)
    d=0.25;x1=x(1);y=x(2);th=x(3);
    
    xs=x1+d*cos(th);
    ys=y+d*sin(th);
    
    f=(ys/0.25)^2-((xs-2)/0.1)^4-1;%>0 forbidden
    df=[ -40*(10*x1 + 10*d*cos(th) - 20)^3, 32*y + 32*d*sin(th), 8*d*cos(th)*(4*y + 4*d*sin(th)) + 40*d*sin(th)*(10*x1 + 10*d*cos(th) - 20)^3, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
end