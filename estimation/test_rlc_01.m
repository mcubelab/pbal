function res=test_rlc_01()




r=20;
a=3;
b=5;
theta=pi/2;

x=a+r*cos(theta);
y=b+r*sin(theta);

obj = recursiveLS(3,'ForgettingFactor',0.95,...
    'InitialParameters',[x,y,-x^2-y^2]);
% obj = recursiveLS(3,'ForgettingFactor',0.99,...
%     'InitialParameters',[a,b,r^2-a^2-b^2]);

fig1=figure(1);

axis square
axis equal
axis(5*r*[-1,1,-1,1]);


hold on
my_pend=line([a,x],[b,y],'linewidth',2,'color','k');
bob1=line(a,b,'Marker','o',...
                'Markerfacecolor','b','Markeredgecolor','b','markersize',5);
bob2=line(x,y,'Marker','o',...
                'Markerfacecolor','r','Markeredgecolor','r','markersize',5);

my_estimate=line(x,y,'Marker','o',...
                'Markerfacecolor','g','Markeredgecolor','g','markersize',5);



for n=1:10000
    theta=2*sin(.1*n);
    
    a=a+.2;
  
    
    x=a+r*cos(theta);
    y=b+r*sin(theta);

    [param_list,EstimatedOutput] = step(obj,x^2+y^2,[2*x,2*y,1]);
    
   
    set(my_pend,'Xdata',[a,x],'Ydata',[b,y]);
    set(bob1,'Xdata',[a],'Ydata',[b]);
    set(bob2,'Xdata',[x],'Ydata',[y]);
    set(my_estimate,'Xdata',param_list(1),'Ydata',param_list(2));
    drawnow;
    
    pause(.01);
end

end