function res=test_rlc_01()




r=20;
a=3;
b=5;
theta=pi/2;

x=a+r*cos(theta);
y=b+r*sin(theta);

% obj = recursiveLS(3,'ForgettingFactor',0.99,...
%     'InitialParameters',[x,y,-x^2-y^2]);
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



alpha=0.01;

xlist=[];
ylist=[];

x0 = 10000;
y0 = 10000;

weight_aggregate=0;
for n=1:10000
    theta=.2*sin(.1*n);
    
%     a=a+.2;
  
    
    x=a+r*cos(theta)+alpha*.5*r*(rand()-.5);
    y=b+r*sin(theta)+alpha*.5*r*(rand()-.5);

    xlist(n)=x;
    ylist(n)=y;
    
    if n>=3

        random_index = randi([1,n],1,3);
        x_random=xlist(random_index);
        y_random=ylist(random_index);

        l1=sqrt((x_random(1)-x_random(2))^2+(y_random(1)-y_random(2))^2);
        l2=sqrt((x_random(2)-x_random(3))^2+(y_random(2)-y_random(3))^2);
        l3=sqrt((x_random(3)-x_random(1))^2+(y_random(3)-y_random(1))^2);



        Triangle_Area = .25*sqrt((l1+l2+l3)*(-l1+l2+l3)*(l1-l2+l3)*(l1+l2-l3));
%         Triangle_Area


        if Triangle_Area>.01
            Triangle_Mat = [-2*x_random',-2*y_random',ones(3,1)];
            Triangle_Vec = -(x_random.^2 + y_random.^2)';
            Coeff_Vec=Triangle_Mat\Triangle_Vec;
            x0_new=Coeff_Vec(1);
            y0_new=Coeff_Vec(2);
            c1=Triangle_Area/(Triangle_Area+weight_aggregate);
            c2=weight_aggregate/(Triangle_Area+weight_aggregate);

            x0=c1*x0_new+c2*x0;
            y0=c1*y0_new+c2*y0;
%             x0=x0_new;
%             y0=y0_new;
            weight_aggregate=.99*(weight_aggregate+Triangle_Area);
        end
        
    end
%     [param_list,EstimatedOutput] = step(obj,x^2+y^2,[2*x,2*y,1]);
    
   
    set(my_pend,'Xdata',[a,x],'Ydata',[b,y]);
    set(bob1,'Xdata',[a],'Ydata',[b]);
    set(bob2,'Xdata',[x],'Ydata',[y]);
    set(my_estimate,'Xdata',x0,'Ydata',y0);
    drawnow;
    
    pause(.01);
end

end