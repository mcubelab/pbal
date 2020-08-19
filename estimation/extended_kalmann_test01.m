function res= extended_kalmann_test01()

figure1=figure(1);


x=10;
dxdt=0;

a=5;
b=5;
c=2;

x_guess=x;
dxdt_guess=dxdt;
a_guess=6;
b_guess=4;
c_guess=2.5;

dt=.01;
omega=5;

count=1;

X=[x;dxdt;a;b;c];
X_guess=[x;dxdt;a_guess;b_guess;c_guess];


R=1*eye(2);
Q=1*eye(5);
P=eye(5);

for t=0:dt:5000*dt

    tlist(count)=t;
    Xlist(:,count)=X;
    X_guess_list(:,count)=X_guess;
   
   
    if(mod(count,10)==0)
        clf;
        
        PlotStateList(tlist,Xlist,'b');
        PlotStateList(tlist,X_guess_list,'r');

        drawnow;
    end

    u=sin(omega*t);

   
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
    x=X_in(1);
    dxdt=X_in(2);
    a=X_in(3);
    b=X_in(4);
    c=X_in(5);
    
    d2xdt2=-a*x-b*dxdt+c*u;
    
    dXdt=[dxdt;d2xdt2;0;0;0];
    
    J=[0,1,0,0,0;...
      -a,-b,-x,-dxdt,u;...
      0,0,0,0,0;
      0,0,0,0,0;
      0,0,0,0,0];
    


end

function [Z,pZpX] = my_output(X_in)
    x=X_in(1);
    dxdt=X_in(2);
    
    Z=[x;dxdt];
    pZpX=[1,0,0,0,0;...
          0,1,0,0,0];


end

