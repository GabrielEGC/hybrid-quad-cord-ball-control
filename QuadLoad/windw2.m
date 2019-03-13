function [f,df]=windw2(x,u)
    xs=x(4);ys=x(5);
    f=(ys/0.5)^2-((xs-2)/0.2)^4-1;%>0 forbidden
    df=[zeros(1,3), -20*(5*xs - 10)^3, 8*ys, zeros(1,9)];
end