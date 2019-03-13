function [f,df]=windw3(x,u)
    l=1;x1=x(1);y=x(2);al=x(6);
    xs=x(1)+l*sin(x(6));ys=x(2)-l*cos(x(6));
    f=(ys/0.25)^2-((xs-2)/0.1)^4-1;%>0 forbidden
    df=[ -40*(10*x1 + 10*l*sin(al) - 20)^3, 32*y - 32*l*cos(al), 0, 0, 0, 8*l*sin(al)*(4*y - 4*l*cos(al)) - 40*l*cos(al)*(10*x1 + 10*l*sin(al) - 20)^3, 0, 0, 0, 0, 0, 0, 0, 0];
end