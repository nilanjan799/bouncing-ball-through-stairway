function []=projectile_simulation()

    %% the following two lines define the stairs 
        x1=sym('x1');
        y2(x1)=piecewise(0<=x1<1,8,1<=x1<2,7.5,2<=x1<3,7,3<=x1<4,6.5,4<=x1<5,6,5<=x1<6,5.5,6<=x1<7,5,7<=x1<8,4.5);
    
 %% function that defines the dynamics of the projectile     
function dz=proj_dyn(t,z)
c=0.1;m=1;g=9.8;  %c-coefficient of qudratic drag; m-mass of the ball; g-acceleration due to gravity
x=z(1);y=z(2);dx=z(3);dy=z(4);
v=sqrt(dx^2+dy^2); %net instantaneous velocity of the ball
fx=-c*v*dx;        %horizntal drag
fy=-c*v*dy-g;      %vertical force
ddx=fx/m;
ddy=fy/m;
dz=[dx;dy;ddx;ddy];

end

%% function for event location for collisions of the ball
function [value,isterminal,direction]=event_function(~,z)
x=z(1,:);
y=z(2,:);
value=y-y2(x);
isterminal= true;
direction=-1;
end

%%
tend=10;tnow=0;
num_bounce=4;     %number of allowable bounces
e=0.9;            %coefficient of restitution
z0=[0;9;1;6];     % z0-state at the start of each trajectory
options=odeset('RelTol',1e-8,'AbsTol',1e-8,'Event',@event_function);   %adds error tolerances and event location properties during the simulation

%% most important part of the whole code
% this loop simulates the entire trajectory of the ball
% each loop corresponds to the motion of the ball from one collision to the
% next
for k=1:num_bounce
   tspan=[tnow,tend];           %simulation time span
   [t,z]=ode45(@proj_dyn,tspan,z0,options);  
   
   %resetting the initial state after collision
   z1=z(end,:);
   z1(4)=-e*z1(4);    %setting the normal velocity component during collision
   z0=z1';
   tnow=t(end);
   
   %unpacking the solution set for plotting and animation purposes
   z=z';
xsol=z(1,:);
ysol=z(2,:);

%plotting
plot(xsol,ysol,'r')
hold on
end

%%
fplot(y2)  %plotting the stairway

end


