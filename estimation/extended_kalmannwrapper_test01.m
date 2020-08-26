function res= extended_kalmannwrapper_test01()

figure1=figure(1);

figure2=figure(2);

x=-pi/2;
dxdt=0;

a=7;
b=5;
theta_0=pi/4;
x_c=1;
y_c=-2;
R=8;

x_guess=0;
dxdt_guess=.5;
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
X_guess=[x_guess;dxdt_guess;a_guess;b_guess;theta_0_guess;x_c_guess;y_c_guess;R_guess];


R=.1*eye(4);
Q=.1*eye(8);
P=.1*eye(8);


theta_range=0:.01:2*pi;


params.m=1;
params.l=1;
params.t_m=1;
params.g=1;
params.mu=1;
params.I=1;
myPlant=PendulumPlant01(params);


for t=0:dt:10000*dt
    Z = myPlant.my_KalmannOutputNoPartials(X);
    Z_guess = myPlant.my_KalmannOutputNoPartials(X_guess);
    
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
        plot(x_c+a*[0,cos(theta_0+pi/2)],y_c+a*[0,sin(theta_0+pi/2)],'r');
        plot(x_c+X(8)*[0,cos(pi/2)],y_c+X(8)*[0,sin(pi/2)],'k');
        plot(x_c+X(8)*[0,cos(X(1)+pi/2)],y_c+X(8)*[0,sin(X(1)+pi/2)],'b')
        

        
        plot(x_c+a*[cos(theta_0+pi/2)],y_c+a*[sin(theta_0+pi/2)],'ko','markerfacecolor','k','markersize',3);
        plot(x_c+X(8)*[cos(pi/2)],y_c+X(8)*[sin(pi/2)],'ko','markerfacecolor','k','markersize',3);
        plot(x_c+X(8)*[cos(X(1)+pi/2)],y_c+X(8)*[sin(X(1)+pi/2)],'ko','markerfacecolor','k','markersize',3)
        plot(x_c,y_c,'ko','markerfacecolor','k','markersize',3)
        plot(X(6)+X(8)*cos(theta_range),X(7)+X(8)*sin(theta_range),'k')
        
        plot(X_guess(6)+X_guess(3)*[0,cos(X_guess(5)+pi/2)],X_guess(7)+X_guess(3)*[0,sin(X_guess(5)+pi/2)],'c');
        plot(X_guess(6)+X_guess(3)*[0,cos(X_guess(5)+pi/2)],X_guess(7)+X_guess(3)*[0,sin(X_guess(5)+pi/2)],'ko','markerfacecolor','k','markersize',3);
        plot(X_guess(6)+X_guess(8)*[0,cos(X_guess(1)+pi/2)],X_guess(7)+X_guess(8)*[0,sin(X_guess(1)+pi/2)],'g')
        plot(X_guess(6)+X_guess(8)*[0,cos(X_guess(1)+pi/2)],X_guess(7)+X_guess(8)*[0,sin(X_guess(1)+pi/2)],'ko','markerfacecolor','k','markersize',3)
        plot(X_guess(6)+X_guess(8)*cos(theta_range),X_guess(7)+X_guess(8)*sin(theta_range),'r')
        
        drawnow;
    end

    u=-3*X(1)-.3*X(2)+(X_guess(3)/X_guess(4))*sin(X_guess(5));
%      u=-2*X(1)-.3*X_guess(2);
%     u=.3*sin(2*t)-.0001*X_guess(2);
%     u=0;
    [dXdt,dXdt_guess,dPdt]= extended_kalmann_update(X,X_guess,u,P,Q,R,myPlant);
    
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


function [dXdt,dXdt_guess,dPdt]= extended_kalmann_update(X,X_guess,u,P,Q,R,myPlant)

    dXdt = myPlant.my_KalmannPlantNoPartials(X,u);
    Z = myPlant.my_KalmannOutputNoPartials(X);
    
    [dXdt_guess_star,F] = myPlant.my_KalmannPlantWithPartials(X_guess,u);
    [Z_guess,H] = myPlant.my_KalmannOutputWithPartials(X_guess);
    
    K=P*H'/R;
    dPdt=F*P+P*F'-K*H*P+Q;

    dXdt_guess=dXdt_guess_star+K*(Z-Z_guess);
end



