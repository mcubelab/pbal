function res= extended_kalmannwrapper_test01()

addpath('../MPCCode/Plants/*')

figure1=figure(1);
figure2=figure(2);

%true parameters
x=-pi/2;
dxdt=0;
a=7;
b=1;
theta_0=0;
x_c=1;
y_c=-2;
R=8;
X=[x;dxdt;a;b;theta_0;x_c;y_c;R];

% inital guess
x_guess= x+1.5*(rand()-.5);
dxdt_guess=dxdt+0*(rand()-.5);
a_guess=a+2*(rand()-.5);
b_guess=b+.1*(rand()-.5);
theta_0_guess=theta_0+.3*(rand()-.5);
x_c_guess=x_c+4*(rand()-.5);
y_c_guess=y_c+4*(rand()-.5);
R_guess=R+.3*(rand()-.5);
X_guess=[x_guess;dxdt_guess;a_guess;b_guess;theta_0_guess;x_c_guess;y_c_guess;R_guess];
% X_guess=X;

% kalman pendulum plant
% params_guess.l = 1; % length
% params_guess.g= (2/3)*a_guess;
% params_guess.m= 3/b_guess;
% params_guess.t_m = params_guess.m * params_guess.g * params_guess.l; % torque limit on input
% params_guess.b = 0.0;  % damping
% params_guess.mu = 0.3; % coefficient of friction
% params_guess.contact_point = [0;-5];

params.g=10;            % gravity (m/s^2)
params.l_contact=1;
l_cm=params.g/(a*b);

params.m=a/(params.g*l_cm);      % mass  (kg)
params.I_cm=0; % moment of inertia about center of mass;
params.t_m=100;   % control torque limit (N*m)


params.mu_pivot=10;     % coefficient of friction at obj/ground contact
params.mu_contact=10;   % coefficient of friction at obj/robot contact
params.Nmax_pivot=100;   % maximum force the ground can exert on obj along contact normal
params.Nmax_pivot=100; % maximum force the robot can exert on obj along contact normal
%             obj.l_contact = params.l_contact;     % length of object/robot contact
params.contact_normal=[1;0]; % direction of the contact normal in the body frame


params.contact_point=R*[1;0];                   %location of contact point in the body frame
params.r_cm=l_cm*[cos(theta_0);sin(theta_0)];    %location of center of mass in the body frame


params_guess.g=10;            % gravity (m/s^2)
l_cm_guess=params_guess.g/(a_guess*b_guess);

params_guess.m=a/(params_guess.g*l_cm_guess);       % mass  (kg)
params_guess.I_cm=0; % moment of inertia about center of mass;
params_guess.t_m=100;   % control torque limit (N*m)

params_guess.l_contact=1;

params_guess.mu_pivot=10;     % coefficient of friction at obj/ground contact
params_guess.mu_contact=10;   % coefficient of friction at obj/robot contact
params_guess.Nmax_pivot=100;   % maximum force the ground can exert on obj along contact normal
params_guess.Nmax_pivot=100; % maximum force the robot can exert on obj along contact normal
%             obj.l_contact = params.l_contact;     % length of object/robot contact
params_guess.contact_normal=[1;0]; % direction of the contact normal in the body frame


params_guess.contact_point=R_guess*[1;0];                   %location of contact point in the body frame
params_guess.r_cm=l_cm_guess*[cos(theta_0_guess);sin(theta_0_guess)];    %location of center of mass in the body frame


p_guess = PyramidPlant01(params_guess);
p_guess.setPivot(x_c_guess,y_c_guess);

% true plant
% params.l = 1; % length
% params.g= (2/3)*a;
% params.m= 3/b;
% params.t_m = params.m * params.g * params.l; % torque limit on input
% params.b = 0.0;  % damping
% params.mu = 0.3; % coefficient of friction
% params.contact_point = R*[sin(theta_0);-cos(theta_0)];
p = PyramidPlant01(params);
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
    u=[0;0;-3*X(1)-.3*X(2)-(X_guess(3)/X_guess(4))*sin(X_guess(5))];
%     u=0;
    
%     uk = [2; 1; 0]; 
    [xkp1, ~] =  p.dynamics_solve(xk, u, dt);
    
    
    Z = p.my_KalmannOutputNoPartials(X);
%     [dXdt,~,~]= extended_kalmann_update(X,X_guess,u,P,Q,R,p);
    
    [dXdt_guess,dPdt]= p_guess.extended_kalmann_update(Z,X_guess,u,P,Q,R);
    
    P=P+dt*dPdt;
%     X_guess=X_guess+dt*dXdt_guess;
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



