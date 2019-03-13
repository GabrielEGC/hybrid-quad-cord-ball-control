function [f,df]=windw5(x,u)
    d=0.25;x1=x(1);y=x(2);th=x(3);
    
    xs=x1-d*cos(th);
    ys=y-d*sin(th);
    
    f=(ys/0.5)^2-((xs-2)/0.2)^4-1;%>0 forbidden
    df=[ 20*(5*d*cos(th) - 5*x1 + 10)^3, 8*y - 8*d*sin(th), 20*d*sin(th)*(5*d*cos(th) - 5*x1 + 10)^3 - 4*d*cos(th)*(2*y - 2*d*sin(th)), 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
end