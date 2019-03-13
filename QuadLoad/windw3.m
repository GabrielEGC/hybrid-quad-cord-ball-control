function [f,df]=windw3(x,u)
    l=2;x1=x(1);y=x(2);al=x(6);
    xs=x(1)+l*sin(x(6));ys=x(2)-l*cos(x(6));
    f=(ys/0.5)^2-((xs-2)/0.2)^4-1;%>0 forbidden
    df=[ -20*(5*x1 + 5*l*sin(al) - 10)^3, 8*y - 8*l*cos(al), 0,0,0, 4*l*sin(al)*(2*y - 2*l*cos(al)) - 20*l*cos(al)*(5*x1 + 5*l*sin(al) - 10)^3,zeros(1,8)];
end