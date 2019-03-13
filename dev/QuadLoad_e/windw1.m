function [f,df]=windw1(x,u)
    xs=x(1);ys=x(2);
    f=(ys/0.25)^2-((xs-2)/0.1)^4-1;%>0 forbidden
    df=[ -40*(10*xs - 20)^3, 32*ys, zeros(1,12)];
end