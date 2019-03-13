function [f,df]=windw2(x,u)
    xs=x(4);ys=x(5);
    f=(ys/0.25)^2-((xs-2)/0.1)^4-1;%>0 forbidden
    df=[zeros(1,3), -40*(10*xs - 20)^3, 32*ys, zeros(1,9)];
end