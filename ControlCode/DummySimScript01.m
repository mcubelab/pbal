%this script tests basic functionality of the MyPolygon and
%PolygonRigidBody classes

function res=DummySimScript01()

fig1=figure(1);
hold on

l=20;
axis equal
axis([-l,l,-l,l])


M=10;
I=0;

plist=[0 , 10, 10;  ...
       0 , -5,  5];
   
r_cm=[20;0];

test_rigid_body=PolygonRigidBody(plist, r_cm, M, I);


test_rigid_body.initialize_visualization();
plot(0,0,'ro','markerfacecolor','r');
% 
% for theta=0:.01:2*pi
%    test_rigid_body.set_p_and_theta([0;0],theta);
%    test_rigid_body.update_visualization();
%    drawnow;
% end
% 
% for theta=0:.01:2*pi
%    test_rigid_body.set_p_and_theta([5*sin(theta);0],0);
%    test_rigid_body.update_visualization();
%    drawnow;
% end
% 
% for theta=0:.01:2*pi
%    test_rigid_body.set_p_and_theta([0;5*sin(theta)],0);
%    test_rigid_body.update_visualization();
%    drawnow;
% end


dt=0.01;

theta=-pi/2+.3;

pivot=[0;0];

pout_temp=PolygonMath.rigid_body_position([0;0],theta,pivot);

position=-pout_temp;

[x,y,Dx,Dy,Hx,Hy]=PolygonMath.rigid_body_position_derivatives(position,theta,pivot);

omega=0;
dgeneralized=[Dx;Dy;[0,0,1]]\[0;0;omega];

generalized=[position;theta];

g=-100;
for count=1:10000

    [x,y,Dx,Dy,Hx,Hy]=PolygonMath.rigid_body_position_derivatives([generalized(1);generalized(2)],generalized(3),pivot);
    
    [x_cm,y_cm,Dx_cm,Dy_cm,Hx_cm,Hy_cm]=PolygonMath.rigid_body_position_derivatives([generalized(1);generalized(2)],generalized(3),r_cm);
    Q1=M*Dx_cm*Dx_cm'+M*Dy_cm*Dy_cm'+I*[0,0,0;0,0,0;0,0,1]
    Q2=M*(Hx_cm*dgeneralized)'*Dx_cm'+M*(Hy_cm*dgeneralized)'*Dy_cm';
    Q2=-(Q2+Q2')*dgeneralized;
    BigMat=[[Q1,-Dx',-Dy'];[Dx,0,0];[Dy,0,0]];
    BigVec=[g*Dy_cm'+Q2;-dgeneralized'*Hx*dgeneralized;-dgeneralized'*Hy*dgeneralized];
    Accel=BigMat\BigVec;
    Accel_generalized=Accel(1:3);

   dgeneralized=dgeneralized+Accel_generalized*dt;
    
   generalized=generalized+dgeneralized*dt;

 
   for n=1:10
       [x,y,Dx,Dy,Hx,Hy]=PolygonMath.rigid_body_position_derivatives([generalized(1);generalized(2)],generalized(3),pivot);
       generalized=generalized-[Dx;Dy]\[x;y];
   end

   [x,y,Dx,Dy,Hx,Hy]=PolygonMath.rigid_body_position_derivatives([generalized(1);generalized(2)],generalized(3),pivot);
    
   temp_mat=[[eye(3),-Dx',-Dy'];[Dx,0,0];[Dy,0,0]];
   temp_vec=[0;0;0;-Dx*dgeneralized;-Dy*dgeneralized];
   
   temp_generalized=temp_mat\temp_vec;
   
   
   dgeneralized=dgeneralized+temp_generalized(1:3);
   
   test_rigid_body.set_p_and_theta([generalized(1);generalized(2)],generalized(3));
   test_rigid_body.update_visualization();
   drawnow;
   
   
   
end





end