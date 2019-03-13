function [f,df]=windw1(x,u)
    xs=x(1);ys=x(2);
    f=(ys/0.5)^2-((xs-2)/0.2)^4-1;%>0 forbidden
    df=[ -20*(5*xs - 10)^3, 8*ys, zeros(1,10)];
end