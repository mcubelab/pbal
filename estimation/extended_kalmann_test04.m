function res= extended_kalmann_test03()

figure1=figure(1);

figure2=figure(2);

x=-pi/2;
dxdt=.3;


x_c=3;
y_c=-1;
R=5;


x_c_guess=0;
y_c_guess=0;
R_guess=7;

dt=.01;
omega=5;

count=1;

X=[x;dxdt;x_c;y_c;R];
X_guess=[x+pi;dxdt-1;x_c_guess;y_c_guess;R_guess];


R=.1*eye(4);
Q=.1*eye(5);
P=.1*eye(5);


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
        hold on
        axis(10*[-1,1,-1,1]);
        axis square;
        
        
        plot(X(3),X(4),'ko','markerfacecolor','k','markersize',3)
        plot(X_guess(3),X_guess(4),'ro','markerfacecolor','r','markersize',3)
        
        
        plot(X(3)+X(5)*cos(theta_range),X(4)+X(5)*sin(theta_range),'k')
        plot(X_guess(3)+X_guess(5)*cos(theta_range),X_guess(4)+X_guess(5)*sin(theta_range),'r')
        plot(Z(1),Z(2),'ko','markerfacecolor','k','markersize',3)
        plot(Z_guess(1),Z_guess(2),'ro','markerfacecolor','r','markersize',3)
%         axis equal;
       

        drawnow;
    end

%     u=-.5*X(1)-.2*X(2)+(X_guess(3)/X_guess(4))*sin(X_guess(5));
%      u=-2*X(1)-.3*X_guess(2);
%     u=.4*sin(t)-.3*X(2);
    u=0;
    
    [dXdt,dXdt_guess,dPdt]= extended_kalmann_update(X,X_guess,u,P,Q,R);
    
    P=P+dt*dPdt;
    X=X+dt*dXdt;
    X_guess=X_guess+dt*dXdt_guess;
   
    
    count=count+1;
end



end


function PlotStateList(tlist,Xlist,mycolor)

    for count=1:5
        subplot(5,1,count);
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
    x_c=X_in(3);
    y_c=X_in(4);
    R=X_in(5);
    
   
    
    d2theta_dt2=0;
    
    dXdt=[dtheta_dt;0;0;0;0];
    
    J=zeros(5);
    
    J(1,2)=1;
    
end

function [Z,pZpX] = my_output(X_in)
    theta=X_in(1);
    dtheta_dt=X_in(2);
    x_c=X_in(3);
    y_c=X_in(4);
    R=X_in(5);
    
    
    x=R*cos(theta)+x_c;
    y=R*sin(theta)+y_c;
    
    dx=-R*sin(theta)*dtheta_dt;
    dy=R*cos(theta)*dtheta_dt;
    
    Z=[x;y;dx;dy];
    
    pZpX=[-R*sin(theta),0           ,1,0,cos(theta);...
          0            ,R*sin(theta),0,1,sin(theta);...
          -R*cos(theta)*dtheta_dt,-R*sin(theta),0,0,-sin(theta)*dtheta_dt;...
          -R*sin(theta)*dtheta_dt, R*cos(theta),0,0, cos(theta)*dtheta_dt];
    
%     Z=[theta;dtheta_dt];
%     pZpX=[1,0,0,0,0;...
%           0,1,0,0,0];


end

