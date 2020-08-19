function res= extended_kalmann_test03()

figure1=figure(1);

figure2=figure(2);

x=-pi/2;
dxdt=0;

a=5;
b=5;
theta_0=pi/4;
x_c=1;
y_c=-2;
R=5;

x_guess=x;
dxdt_guess=dxdt;
a_guess=6;
b_guess=2;
theta_0_guess=0;
x_c_guess=0;
y_c_guess=0;
R_guess=6;

dt=.01;
omega=5;

count=1;

X=[x;dxdt;a;b;theta_0;x_c;y_c;R];
X_guess=[x;dxdt;a_guess;b_guess;theta_0_guess;x_c_guess;y_c_guess;R_guess];


R=.1*eye(6);
Q=.1*eye(8);
P=.1*eye(8);


theta_range=0:.01:2*pi;
for t=0:dt:10000*dt
    [Z,~] = my_output(X);
    [Z_guess,~] = my_output(X_guess);
    
    tlist(count)=t;
    Xlist(:,count)=X;
    X_guess_list(:,count)=X_guess;
   
   
    if(mod(count,20)==0)
        
        set(0,'currentfigure',figure1);
        clf;
        PlotStateList(tlist,Xlist,'b');
        PlotStateList(tlist,X_guess_list,'r');
        drawnow;
        
        set(0,'currentfigure',figure2);
        clf;
        axis(10*[-1,1,-1,1]);
        axis square;
%         axis equal;
        
        hold on;
        plot(x_c+X(8)*[0,cos(theta_0+pi/2)],y_c+X(8)*[0,sin(theta_0+pi/2)],'r');
        plot(x_c+X(8)*[0,cos(pi/2)],y_c+X(8)*[0,sin(pi/2)],'k');
        plot(x_c+X(8)*[0,cos(X(1)+pi/2)],y_c+X(8)*[0,sin(X(1)+pi/2)],'b')
        
        
        
        plot(x_c+X(8)*[cos(theta_0+pi/2)],y_c+X(8)*[sin(theta_0+pi/2)],'ko','markerfacecolor','k','markersize',3);
        plot(x_c+X(8)*[cos(pi/2)],y_c+X(8)*[sin(pi/2)],'ko','markerfacecolor','k','markersize',3);
        plot(x_c+X(8)*[cos(X(1)+pi/2)],y_c+X(8)*[sin(X(1)+pi/2)],'ko','markerfacecolor','k','markersize',3)
        plot(x_c,y_c,'ko','markerfacecolor','k','markersize',3)
        plot(X(6)+X(8)*cos(theta_range),X(6)+X(8)*sin(theta_range),'k')
        
        plot(X_guess(6)+X_guess(8)*[0,cos(X_guess(1)+pi/2)],X_guess(7)+X_guess(8)*[0,sin(X_guess(1)+pi/2)],'g')
        plot(X_guess(6)+X_guess(8)*[0,cos(X_guess(1)+pi/2)],X_guess(7)+X_guess(8)*[0,sin(X_guess(1)+pi/2)],'ko','markerfacecolor','k','markersize',3)
        plot(X_guess(6)+X_guess(8)*cos(theta_range),X_guess(6)+X_guess(8)*sin(theta_range),'r')
        
        drawnow;
    end

    u=-3*X(1)-.3*X(2)+(X_guess(3)/X_guess(4))*sin(X_guess(5));
%      u=-2*X(1)-.3*X_guess(2);
%     u=.4*sin(t)-.3*X(2);
    
    [dXdt,dXdt_guess,dPdt]= extended_kalmann_update(X,X_guess,u,P,Q,R);
    
    P=P+dt*dPdt;
    X=X+dt*dXdt;
    X_guess=X_guess+dt*dXdt_guess;
     
    
    count=count+1;
end



end


function PlotStateList(tlist,Xlist,mycolor)

    for count=1:8
        subplot(8,1,count);
        hold on
        plot(tlist,Xlist(count,:),'color',mycolor);
    end

    
end


function [dXdt,dXdt_guess,dPdt]= extended_kalmann_update(X,X_guess,u,P,Q,R)

    [dXdt,~] = my_plant(X,u);
    [Z,~] = my_output(X);
    
    [dXdt_guess_star,F] = my_plant(X_guess,u);
    [Z_guess,H] = my_output(X_guess);
    

    
    K=P*H'/R;
    dPdt=F*P+P*F'-K*H*P+Q;

    dXdt_guess=dXdt_guess_star+K*(Z-Z_guess);
    
end


function [dXdt,J] = my_plant(X_in,u)
    theta=X_in(1);
    dtheta_dt=X_in(2);
    a=X_in(3);
    b=X_in(4);
    theta_0=X_in(5);
    x_c=X_in(6);
    y_c=X_in(7);
    R=X_in(8);
    
    d2theta_dt2=a*sin(theta-theta_0)+b*u;
    
    dXdt=[dtheta_dt;d2theta_dt2;0;0;0;0;0;0];
    
    J=zeros(8);
    
    J(1,2)=1;
    J(2,1:5)=[a*cos(theta-theta_0),0,sin(theta-theta_0),u,-a*cos(theta-theta_0)];
    
end

function [Z,pZpX] = my_output(X_in)
    theta=X_in(1);
    dtheta_dt=X_in(2);
    a=X_in(3);
    b=X_in(4);
    theta_0=X_in(5);
    x_c=X_in(6);
    y_c=X_in(7);
    R=X_in(8);
    
    x=R*cos(theta)+x_c;
    y=R*sin(theta)+y_c;
    
    dx=-R*sin(theta)*dtheta_dt;
    dy=R*cos(theta)*dtheta_dt;
    
        
%     Z=[x;y;dx;dy];
%     
%     pZpX=[-R*sin(theta),0           ,0,0,0,1,0,cos(theta);...
%           0            ,R*sin(theta),0,0,0,0,1,sin(theta);...
%           -R*cos(theta)*dtheta_dt,-R*sin(theta),0,0,0,0,0,-sin(theta)*dtheta_dt;...
%           -R*sin(theta)*dtheta_dt,R*cos(theta),0,0,0,0,0, cos(theta)*dtheta_dt];
    
      
%     
    Z=[x;y;dx;dy;theta;dtheta_dt];
    
    pZpX=[-R*sin(theta),0           ,0,0,0,1,0,cos(theta);...
          0            ,R*sin(theta),0,0,0,0,1,sin(theta);...
          -R*cos(theta)*dtheta_dt,-R*sin(theta),0,0,0,0,0,-sin(theta)*dtheta_dt;...
          -R*sin(theta)*dtheta_dt,R*cos(theta),0,0,0,0,0, cos(theta)*dtheta_dt;
          1,0,0,0,0,0,0,0;...
          0,1,0,0,0,0,0,0];
%     
%     Z=[theta;dtheta_dt];
%     pZpX=[1,0,0,0,0;...
%           0,1,0,0,0];


end

