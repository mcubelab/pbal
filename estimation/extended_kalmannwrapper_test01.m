function res= extended_kalmannwrapper_test01()

figure1=figure(1);
figure2=figure(2);

%true parameters
x=-pi/2;
dxdt=0;
a=7;
b=5;
theta_0=0;
x_c=1;
y_c=-2;
R=8;
X=[x;dxdt;a;b;theta_0;x_c;y_c;R];

% inital guess
x_guess= x+pi/2;
dxdt_guess=0.2;
a_guess=7-1;
b_guess=5+1;
theta_0_guess=pi/6;
x_c_guess=1-0.2;
y_c_guess=-2+0.2;
R_guess=8-1;
X_guess=[x_guess;dxdt_guess;a_guess;b_guess;theta_0_guess;x_c_guess;y_c_guess;R_guess];
% X_guess=X;

% kalman pendulum plant
params_guess.l = 1; % length
params_guess.g= (2/3)*a_guess;
params_guess.m= 3/b_guess;
params_guess.t_m = params_guess.m * params_guess.g * params_guess.l; % torque limit on input
params_guess.b = 0.0;  % damping
params_guess.mu = 0.3; % coefficient of friction
p_guess = PendulumPlant01(params_guess);
p_guess.setPivot(x_c_guess,y_c_guess);

% true plant
params.l = 1; % length
params.g= (2/3)*a;
params.m= 3/b;
params.t_m = params.m * params.g * params.l; % torque limit on input
params.b = 0.0;  % damping
params.mu = 0.3; % coefficient of friction
p = PendulumPlant01(params);
p.setPivot(x_c,y_c);


R=.1*eye(4);
Q=.01*eye(8);
P=.1*eye(8);

xk = [x_c; y_c; x; 0; 0; dxdt]; % true initial state
xk_guess = [x_c_guess; y_c_guess; x_guess; 0; 0; dxdt_guess]; % guess initial state

dt=.01;

theta_range=0:.01:2*pi;
count=1;
for t=0:dt:10000*dt    
    tlist(count)=t;
    Xlist(:,count)=X;
    X_guess_list(:,count)=X_guess;

    
    if(mod(count,200)==0)
        
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

%     u=-3*X(1)-.3*X(2)-(X_guess(3)/X_guess(4))*sin(X_guess(5));
%      u=-2*X(1)-.3*X_guess(2);
    u=.1*sin(2*t)-.01*X_guess(2);
%     u=0;
    
    uk = [0; 0; u]; 
    [xkp1, ~] =  p.dynamics_solve(xk, uk, dt);
    
    
    Z = p.my_KalmannOutputNoPartials(X);
%     [dXdt,~,~]= extended_kalmann_update(X,X_guess,u,P,Q,R,p);
    
    [dXdt_guess,dPdt]= p_guess.extended_kalmann_update(Z,X_guess,u,P,Q,R);
    
    P=P+dt*dPdt;
    X_guess=X_guess+dt*dXdt_guess;
    

    xk=xkp1;
    X(1) = xkp1(3) + X(5);
    X(2) = xkp1(6);
    
%     X=X+dt*dXdt;
    
    
    

    
    
    
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



