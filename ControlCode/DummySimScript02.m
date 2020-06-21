%this script tests basic functionality of the MyPolygon and
%PolygonRigidBody classes

function res=DummySimScript02()

SimulationTest02()

end

%Just visualizing a polygon using the simulation framework
function res=VisualizationTest01()

fig1=figure(1);
hold on

l=20;
axis equal
axis([-l,l,-l,l])


plist1=[0 , 10, 10;  ...
       0 , -5,  5];
r_cm1=[4;0];
   
plist2=[0 ,0 , 5, 5;  ...
       -5, 5 , 5,  -5];
   
r_cm2=[2.5;0];

M=10;
I=100;

test_rigid_body1=PolygonRigidBody(plist1, r_cm1, M, I);
test_rigid_body2=PolygonRigidBody(plist2, r_cm2, M, I);


rand_pos1=l*(rand(2,1)-.5);
rand_theta1=rand(1)*2*pi;

rand_pos2=l*(rand(2,1)-.5);
rand_theta2=rand(1)*2*pi;

test_rigid_body1.set_p_and_theta(rand_pos1,rand_theta1);
test_rigid_body2.set_p_and_theta(rand_pos2,rand_theta2);

test_constraint1=PolygonConstraint();

test_constraint1.StickingContactOneBody(test_rigid_body1,[10;0],[0;0]);

test_constraint2=PolygonConstraint();

test_constraint2.StickingContactTwoBodies(test_rigid_body1,plist1(:,2),test_rigid_body2,plist2(:,3));

myEnvironment=SimulationEnvironment();

myEnvironment.addRigidBody(test_rigid_body1);
myEnvironment.addRigidBody(test_rigid_body2);
myEnvironment.addConstraint(test_constraint1);
myEnvironment.addConstraint(test_constraint2);

myEnvironment.initialize_visualization();

myEnvironment.ConstraintProjection();
drawnow;
pause(10)
myEnvironment.update_visualization();
drawnow;
end

function res=SimulationTest01()

fig1=figure(1);
hold on

g=[0;-10];

l=20;
axis equal
axis([-l,l,-l,l])


plist1=[0 , 10, 10;  ...
       0 , -5,  5];
r_cm1=[4;0];
   

M=10;
I=100;

test_rigid_body1=PolygonRigidBody(plist1, r_cm1, M, I);

myGravity1=PolygonGeneralizedForce();
myGravity1.gravity(test_rigid_body1,g);

test_constraint1=PolygonConstraint();
test_constraint1.StickingContactOneBody(test_rigid_body1,[00;0],[0;0]);

myEnvironment=SimulationEnvironment();
myEnvironment.setdt(.01);

myEnvironment.addRigidBody(test_rigid_body1);
myEnvironment.addConstraint(test_constraint1);
myEnvironment.addGeneralizedForce(myGravity1);

myEnvironment.ConstraintProjection();
myEnvironment.initialize_visualization();


for n=1:10000
    myEnvironment.update_project_visualize();
    drawnow;
end


end



%Just visualizing a polygon using the simulation framework
function res=SimulationTest02()

fig1=figure(1);
hold on

g=[0;-10];

l=20;
axis equal
axis([-l,l,-l,l])


plist1=[0 , 10, 10;  ...
       0 , -5,  5];
r_cm1=[4;0];
   
plist2=[0 ,0 , 5, 5;  ...
       -5, 5 , 5,  -5];
   
r_cm2=[2.5;0];

M=10;
I=100;

test_rigid_body1=PolygonRigidBody(plist1, r_cm1, M, I);
test_rigid_body2=PolygonRigidBody(plist2, r_cm2, M, I);


rand_pos1=l*(rand(2,1)-.5);
rand_theta1=rand(1)*2*pi;

rand_pos2=l*(rand(2,1)-.5);
rand_theta2=rand(1)*2*pi;

test_rigid_body1.set_p_and_theta(rand_pos1,rand_theta1);
test_rigid_body2.set_p_and_theta(rand_pos2,rand_theta2);

test_constraint1=PolygonConstraint();

test_constraint1.StickingContactOneBody(test_rigid_body1,[10;0],[0;0]);

test_constraint2=PolygonConstraint();

test_constraint2.StickingContactTwoBodies(test_rigid_body1,plist1(:,2),test_rigid_body2,plist2(:,3));


myGravity1=PolygonGeneralizedForce();
myGravity1.gravity(test_rigid_body1,g);

myGravity2=PolygonGeneralizedForce();
myGravity2.gravity(test_rigid_body2,g);

myEnvironment=SimulationEnvironment();
myEnvironment.setdt(.0001);


myEnvironment.addRigidBody(test_rigid_body1);
myEnvironment.addRigidBody(test_rigid_body2);
myEnvironment.addConstraint(test_constraint1);
myEnvironment.addConstraint(test_constraint2);
myEnvironment.addGeneralizedForce(myGravity2);
myEnvironment.addGeneralizedForce(myGravity2);

myEnvironment.initialize_visualization();

myEnvironment.ConstraintProjection();
drawnow;


myEnvironment.update_visualization();
drawnow;



for n=1:1000000
    myEnvironment.update_project();
    if mod(n,10)==0
        myEnvironment.update_visualization();
        drawnow;
    end

end



end