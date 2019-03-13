function [f,df]=windw2(x,u)
    xs=x(3);ys=x(4);
    f=(ys/0.5)^2-((xs-2)/0.2)^4-1;%>0 forbidden
    df=[zeros(1,2), -20*(5*xs - 10)^3, 8*ys, zeros(1,8)];
end