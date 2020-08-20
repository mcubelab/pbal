%this script tests basic functionality of the MyPolygon and
%PolygonRigidBody classes

function res=DummyRodPendulum01()


% % user specified
m = 10;   % mass (kg)
l = 10;   % pendulum length (m)
I = m*l^2/3; % inertia about pivot (kg^2 * m^2)
I_com = m*l^2/12;
fig1=figure(1);
hold on

l_axis=2*l;
axis equal
axis(l_axis*[-1,1,-1,1])



plist=[0 , 0;  ...
       0 , l];
   
r_cm=[0;l/2];

test_rigid_body1=PolygonRigidBody(plist, r_cm, m, I_com);

dt=0.1;

theta=-pi/2+.3;

pivot=[0;0];

pout_temp=PolygonMath.rigid_body_position([0;0],theta,pivot);

test_rigid_body1.set_p_and_theta(-pout_temp,theta)


[x,y,Dx,Dy,Hx,Hy]=PolygonMath.rigid_body_position_derivatives(test_rigid_body1.LocalPolygon.position,theta,pivot);

omega=0;
dgeneralized=[Dx;Dy;[0,0,1]]\[0;0;omega];




test_rigid_body1.set_v_and_omega(dgeneralized(1:2),dgeneralized(3));

g=[0;-10];

% g=g*0;

myGravity1=PolygonGeneralizedForce();
myGravity1.gravity(test_rigid_body1,g);

myWrench1=PolygonGeneralizedForce();
myWrench1.external_wrench(test_rigid_body1,[0;0]);

test_constraint1=PolygonConstraint();
test_constraint1.StickingContactOneBody(test_rigid_body1,[00;0],[0;0]);

myEnvironment=SimulationEnvironment();
dt=.01;
myEnvironment.setdt(dt);

myEnvironment.addRigidBody(test_rigid_body1);
myEnvironment.addConstraint(test_constraint1);
myEnvironment.addGeneralizedForce(myGravity1);
myEnvironment.addGeneralizedForce(myWrench1);

myEnvironment.ConstraintProjection();
myEnvironment.initialize_visualization();

tau=10;



for count=1:10000

    myWrench1.set_wrench_value([0;0;tau]);
    
    
    v= myEnvironment.build_velocity_vector();
    q = myEnvironment.build_coordinate_vector();
    
    myEnvironment.computeAccelerations();
    a= myEnvironment.build_acceleration_vector()
    
    (tau-.5*m*g(2)*l*sin(q(3)))/(I)
    
    myEnvironment.EulerUpdate();
    myEnvironment.ConstraintProjection();
  
    if mod(count,100)==0
        set(0,'currentfigure',fig1);
        myEnvironment.update_visualization();
        drawnow;

    end
   
   
end





end