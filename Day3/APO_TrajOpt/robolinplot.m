th1 = z(:,4);
th2 = z(:,5);
th3 = z(:,6);
%read l1 and l2 from the previous code
l1 = init.l1;
l2 = init.l2;
plot_bounds = l1+l2+0.1;

%The coordinates of the links of each of the manipulator
x1 = l1*cos(th1);
y1 = l1*sin(th1);
z1 = l1*sin(th2);
x2 = l1*cos(th1)+l2*cos(th1+th3);
y2 = l1*sin(th1)+l2*sin(th1+th3);
z2 = l1*sin(th2)+l2*sin(th2+th3);
figHandle = figure();
for i=1:init.N
    A = [0 x1(i)]; 
    B = [0 y1(i)]; 
    C = [0 z1(i)];
    %   subplot(1,3,1);
    plot3(A,B,C,'*')
    axis([0 plot_bounds -plot_bounds plot_bounds -plot_bounds plot_bounds])
    hold on
    line(A,B,C)
    hold on
    A2 = [x1(i) x2(i)]; 
    B2 = [y1(i) y2(i)];
    C2 = [z1(i) z2(i)];
    %   subplot(1,3,1);
    plot3(A2,B2,C2,'*')
    
    axis([-plot_bounds plot_bounds 0 plot_bounds -plot_bounds plot_bounds])
    hold on
    line(A2,B2,C2,'Color','red')
    pause(0.1);
end