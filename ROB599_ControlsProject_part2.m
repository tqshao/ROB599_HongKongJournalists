function U_ode =  ROB599_ControlsProject_part2_Team22(TestTrack,Xobs)
% credit by ROB590 Team22
tic
% reshape the track 
bl_new = TestTrack.bl;
br_new = TestTrack.br;
cline_new = TestTrack.cline;
theta = TestTrack.theta;

Nobs = length(Xobs);
if Nobs >21
    num_interval = 20;
else
    num_interval = 25;
end

%load('Xobs')
% reshape the track 
bl_obs = bl_new;
br_obs = br_new;
obs_list = [];

for i=1:length(Xobs)
    % obs is the second ccw point of the obstacle, which is on the cline
    obs = Xobs{i}(2,:);
    dis = cline_new-obs';
    % find the closer starting point on the track (to the obstacle)
    [~,index] = min(sum(dis(:,1:end-1).*dis(:,2:end)));
    % if the obstacle is on the left, obs_sides = 1.
    obs_sides = norm(Xobs{i}(2,:)'-cline_new(:,index))  < norm(Xobs{i}(1,:)-cline_new(:,index)');
    obs_list = [obs_list index];
    if obs_sides == 0
        % replace the closer starting point with the first point in Xobs
        br_obs(:,index) = Xobs{i}(1,:)' + 0.04* (Xobs{i}(1,:)'-Xobs{i}(2,:)');
        % insert the end point in Xobs to both bl and br
        br_obs(:,index+1) = Xobs{i}(4,:)' + 0.04*(Xobs{i}(4,:)'-Xobs{i}(3,:)');
        
    else
         bl_obs(:,index) = Xobs{i}(2,:)' + 0.04*(Xobs{i}(2,:)'-Xobs{i}(1,:)');
         bl_obs(:,index+1) = Xobs{i}(3,:)' + 0.04*(Xobs{i}(3,:)' - Xobs{i}(4,:)');
    end
end

%% generate all trajectory
steer_input = 0.5*[hao(0,-0.05);hao(-0.05,-0.04);-0.05*ones(50,1);hao(-0.05,0);0*ones(50,1);0*ones(50,1);0*ones(50,1);0*ones(50,1);...
    -0.005*ones(50,1);-0*ones(50,1);0*ones(50,1);0*ones(50,1);0*ones(50,1);hao(0,-0.05);hao(-0.05,-0.12);hao(-0.12,-0.08);-0.08*ones(50,1);...
  hao(-0.08,0);-0.04*ones(50,1);hao(-0.04,-0.02);hao(-0.02,-0.18);hao(-0.18,-0.02);-0.02*ones(50,1);hao(-0.02,-0.05);hao(-0.05,-0.1);...
 hao(-0.12,-0);hao(0,-0.15);hao(-0.15,-0.06);hao(-0.06,-0.035);-0.035*ones(50,1);0*ones(50,1);0*ones(50,1);0.01*ones(50,1);-0.01*ones(50,1);...
 0.005*ones(50,1);0.*ones(50,1);0*ones(50,1);0.005*ones(50,1);0*ones(50,1);hao(0,0.03);0.03*ones(50,1);hao(0.03,0.1);0.1*ones(50,1);...
 hao(0.1,0.15);hao(0.15,0.1);hao(0.1,0.01);0.01*ones(50,1);hao(0.15,0.01);0.01*ones(50,1);0.01*ones(50,1);hao(0,-0.15);hao(-0.15,-0.05);...
 -0.05*ones(50,1);hao(-0.05,-0.04);hao(-0.04,-0.2);hao(-0.2,-0.25);hao(-0.25,-0.1);hao(-0.1,-0.01);-0.02*ones(50,1);-0.*ones(50,1);...
 hao(0,0.08);0.08*ones(50,1);0.05*ones(50,1);0.08*ones(50,1);hao(0.08,0.12);hao(0.12,0.15);hao(0.15,0.1);0.15*ones(50,1);hao(0.15,0.05);...
 hao(0.05,0.08);0.05*ones(50,1);hao(0.07,-0.01);hao(-0.01,-0.02);hao(-0.02,-0.05);-0.05*ones(50,1);-0.05*ones(50,1);hao(-0.05,-0.1);...
hao(-0.1,-0.05);-0.05*ones(50,1);hao(-0.05,-0.1);-0.04*ones(50,1);hao(-0.04,-0.07);hao(-0.07,-0.03);-0.03*ones(50,1);hao(-0.03,-0.1);...
hao(-0.1,-0.02);-0.02*ones(50,1);-0.02*ones(50,1);hao(-0.02,-0.05);-0.05*ones(50,1);hao(-0.05,-0.15);hao(-0.15,-0.05);-0.05*ones(50,1);...
hao(-0.05,-0.06);-0.06*ones(50,1);hao(-0.06,-0.2);hao(-0.2,-0.08);hao(-0.08,-0.1);hao(-0.1,0);0*ones(50,1);0*ones(50,1);...
0.01*ones(50,1);0.0*ones(50,1);0*ones(50,1);0*ones(50,1);0*hao(1,-2);hao(0,0.06); hao(0.06,0.4);hao(0.4,0.5); 0.4*ones(50,1);...
0.4*ones(50,1);hao(0.4,0.16);0.16*ones(50,1);hao(0.16,0.02);0.02*ones(50,1); hao(0.03,0); hao(0,0.08);hao(0.08,0);hao(0,0.1);...
hao(0.1,0);hao(0,0.01);0.01*ones(50,1);0.011*ones(50,1);0.01*ones(50,1);hao(0,-0.05);-0.06*ones(50,1);hao(-0.06,-0.2);hao(-0.2,-0.15);...
hao(-0.15,-0.05);hao(-0.05,-0.15);hao(-0.15,-0.16);-0.16*ones(50,1);hao(-0.16,-0.05);hao(-0.05,-0.1);hao(-0.1,-0.15);...
hao(-0.15,-0.1);hao(-0.1,-0.08);hao(-0.08,-0.1);hao(-0.1,-0.15);hao(-0.15,-0.2);hao(-0.2,-0.1);hao(-0.1,-0.01);hao(-0.01,-0);...
hao(0,0.1);hao(0.1,0.2);hao(0.2,0.25);hao(0.25,0.26);0.25*ones(50,1);0.25*ones(50,1);hao(0.2,0.3);hao(0.3,0.1);hao(0.1,0.01);...
0*ones(50,1);-0*ones(50,1);0*ones(50,1);0.01*ones(50,1);0*ones(50,1);-0.01*ones(50,1);0*ones(50,1);0*ones(50,1);0*ones(50,1);...
0*ones(50,1);0*ones(50,1);hao(0,0.03);hao(0.03,0.08);hao(0.08,0.2);hao(0.2,0.25);hao(0.25,0.15);hao(0.15,0.02);0*ones(50,1);...
0.01*ones(50,1);0.005*ones(50,1);0*ones(50,1);0*ones(50,1);0*ones(50,1);-0.01*ones(50,1);0*ones(50,1);0.01*ones(50,1);0*ones(50,1);...
0*ones(50,1);  -0.005*ones(50,1);0*ones(50,1); 0*ones(50,1);0.02*ones(50,1);0*ones(50,1);0.01*ones(50,1);0*ones(50,1);0*ones(50,1)
     ];
% steer_input = [-0.000*ones(300,1);-0.2*ones(300,1);-0.5*ones(150,1)];
force_input = 4500*[0*ones(50,1);hao(0,1);ones(50,1);ones(50,1);ones(50,1);ones(50,1);ones(50,1);ones(50,1);ones(50,1);ones(50,1);...
    ones(50,1);hao(1,-2);-2*ones(50,1);-2*ones(50,1);hao(-2,0);0*ones(50,1);0*ones(50,1);0*ones(50,1);0*ones(50,1);0*ones(50,1);0*ones(50,1);...
    0*ones(50,1);0*ones(50,1);0*ones(50,1);0*ones(50,1);hao(0,1);hao(1,0);0*ones(50,1);hao(0,1);ones(50,1);ones(50,1);ones(50,1);...
    ones(50,1);ones(50,1);ones(50,1);ones(50,1);hao(1,-2);-2*ones(50,1);-2*ones(50,1);-2*ones(50,1);hao(-2,0);0*ones(50,1);0*ones(50,1);...
    0*ones(50,1);0*ones(50,1);0*ones(50,1);0*ones(50,1);hao(0,1);ones(50,1);hao(1,-2);-2*ones(10,1);hao(-2,0);0*ones(40,1);0*ones(50,1);...
    0*ones(50,1);0*ones(50,1);0*ones(50,1);0*ones(50,1);hao(0,1);ones(50,1);ones(50,1);hao(1,-2);-2*ones(20,1);jun(-2,0,20);...
    0*ones(50,1);0*ones(50,1);0*ones(50,1);0*ones(50,1);0*ones(50,1);0*ones(50,1);0*ones(50,1);hao(0,1);ones(50,1);ones(50,1);...
    ones(25,1);hao(1,-2);-2*ones(25,1);hao(-2,0);0*ones(50,1);0*ones(50,1);0*ones(50,1);0*ones(50,1);0*ones(50,1);0*ones(50,1);...
    0*ones(50,1);0*ones(50,1);0*ones(50,1);ones(50,1);ones(50,1);hao(1,0);0*ones(50,1);hao(0,-2);hao(-2,0);0*ones(50,1);...
    0*ones(50,1);0*ones(50,1);0*ones(50,1);hao(0,1);hao(1,-2);hao(-2,1);ones(50,1);ones(50,1);ones(50,1);ones(50,1);...
    ones(50,1);ones(50,1);ones(20,1);hao(1,-2);-2*ones(50,1);-2*ones(30,1);    -2*ones(30,1);hao(-2,0);0*ones(20,1);...
    0*ones(50,1);   0*ones(20,1);  jun(0,-2,25);jun(-2,0,5);0*ones(30,1); 0*ones(50,1);hao(0,1);ones(50,1);ones(50,1);...
    ones(50,1); ones(50,1);ones(75,1);hao(1,-2);-2*ones(50,1);0*ones(75,1);0*ones(50,1) ;0*ones(50,1);0*ones(50,1);...
    0*ones(50,1);jun(0,-2,25);jun(-2,0,60);hao(0,1);ones(50,1);hao(1,0);0*ones(45,1);0*ones(50,1);jun(0,-2,10);jun(-2,0,10);...
    0*ones(60,1);0*ones(50,1);0*ones(50,1);0*ones(50,1);0*ones(40,1);hao(0,1);ones(50,1);jun(1,-2,40);jun(-2,0,20);0*ones(10,1);...
    0*ones(20,1);0*ones(50,1);0*ones(50,1);jun(0,-2,5);jun(-2,0,5);0*ones(20,1);0*ones(50,1);0*ones(50,1);0*ones(50,1);hao(0,1);hao(1,0);...
    hao(0,1);ones(50,1);ones(50,1);ones(50,1);ones(150,1);ones(50,1);ones(50,1);hao(1,-2);-2*ones(50,1);-2*ones(50,1);-2*ones(65,1);...
    hao(-2,0);0*ones(35,1);0*hao(0,0.05);0*hao(0,0.05);0*ones(50,1);hao(0,1);ones(50,1);ones(50,1);ones(50,1);...
    ones(50,1);ones(50,1);ones(50,1);ones(50,1);ones(50,1);ones(50,1);ones(50,1);  ones(50,1);ones(50,1);ones(50,1);ones(50,1);ones(50,1)
     ones(50,1);ones(50,1);ones(50,1)                               
   ]; 
U_all = [steer_input force_input];
%U_all = u_all';
initial_states = [287;5;-176;0;2;0];
traj_all = forwardIntegrateControlInput(U_all,initial_states);

%% MPC controller
initial_states = [287;5;-176;0;2;0];
update_states = [287;5;-176;0;2;0];
delta_T = 0.1;
%time = ceil(length(traj_all)/Nobs/100)+1;
time = 6;
steps = time/delta_T;

Ndec = 6*(steps+1) + 2*steps;
u_start = 6*(steps+1)+1;

shift = 1;

U_ode = [];

f = zeros(Ndec,1);


iteration = round((length(traj_all)/100 - time)/shift/delta_T);
validate_time = 5;
traj_record = [];
for n = 1:iteration-1
   %n
traj_ref = traj_all(1+shift*delta_T/0.01*(n-1):delta_T/0.01:time*100+1+shift*delta_T/0.01*(n-1),:);
U_ref = U_all(1+shift*delta_T/0.01*(n-1):delta_T/0.01:time*100+shift*delta_T/0.01*(n-1),:);

% upper and lower bounds
lb = -1000*ones(Ndec,1);
lb(u_start:2:end) = max(-0.5 - U_ref(1:end,1),-0.01);
lb(u_start+1:2:end) = -10000 - U_ref(1:end,2);
ub = 1000*ones(Ndec,1);
ub(u_start:2:end) = min(0.5 - U_ref(1:end,1),0.01);
ub(u_start+1:2:end) = 5000 - U_ref(1:end,2);

% inequality constrains (boundary conditions)
A = zeros(2*(steps+1),Ndec);
b = zeros(2*(steps+1),1);

for i = 3:length(traj_ref)
    coord = [traj_ref(i,1); traj_ref(i,3)];
    %[x_,y_,rhs] = boundary_non(cline_new,bl_obs,coord);
    %[x_,y_,rhs] = boundary_new(bl_obs,coord);
    [x_,y_,rhs] = boundary(bl_obs,coord,obs_list);

    A(i,6*i-5) = x_;
    A(i,6*i-3) = y_;
    b(i) = rhs - 1e-2;
    %[x_,y_,rhs] = boundary_non(cline_new,br_obs,coord);
    %[x_,y_,rhs] = boundary_new(br_obs,coord);
    [x_,y_,rhs] = boundary(br_obs,coord,obs_list);


    A(steps+1+i,6*i-5) = -x_;
    A(steps+1+i,6*i-3) = -y_;
    b(steps+1+i) = -rhs - 1e-2;
end


% equality constrains
Aeq = zeros(6*steps+6,Ndec);
beq = zeros(6*steps+6,1);
Aeq(1:6,1:6) = eye(6);
beq(1:6) = initial_states - traj_ref(1,:)';

for i = 1:steps
    Jac = Jdf(U_ref(i,1),U_ref(i,2),traj_ref(i,:));
    A0 = Jac(:,3:end);
    B0 = Jac(:,1:2);
    Aeq(6*i+1:6*i+6,6*i-5:6*i) = A0*delta_T+eye(6);
    Aeq(6*i+1:6*i+6,6*i+1:6*i+6) = -eye(6);
    Aeq(6*i+1:6*i+6,u_start+2*i-2:u_start+2*i-1) = B0*delta_T;
end

H = zeros(Ndec,Ndec);
for i=1:steps
    H(6*i-5:6*i,6*i-5:6*i) = diag([4,4,1,1,1,1]);
end

%opts = optimoptions('quadprog','Algorithm','interior-point-convex');
x = quadprog(H,f,A,b,Aeq,beq,lb,ub);

% !!!!!change initial states!!!!!!!!!!
U_steer = x(u_start:2:end);
U_force = x(u_start+1:2:end);
U_delta = interp1(0:delta_T:time-delta_T,[U_steer U_force],0:0.01:shift*delta_T);%,'previous','extrap');
% not interpolation, copy .....
%U_delta = ones(shift*delta_T*100+1,2).*[U_steer(1) U_force(1)];
U_test = U_delta + U_all(1+shift*(n-1)*delta_T*100:size(U_delta,1)+shift*(n-1)*delta_T*100,:);
for i = 1:length(U_test)
    if U_test(i,2)>5000
        U_test(i,2) = 5000;
    elseif U_test(i,2)<-10000
        U_test(i,2) = -10000;
    end
end
U_ode = [U_ode;U_test(1:end-1,:)];
% calculate final states
%initial_states = x(1+6*shift:6+6*shift) + traj_ref(1+shift,:)';

traj_uler(1,:) = initial_states';
for i = 1:length(U_test)-1
    traj_uler(i+1,:) = forward(U_test(i,:),traj_uler(i,:));
end
initial_states = traj_uler(end,:)';

%traj = forwardIntegrateControlInput(U_test,initial_states);
%traj = forwardIntegrateControlInput([U_ode;U_test(end,:)],[287;5;-176;0;2;0]);

if ( mod(n,num_interval) ==0 || iteration == n+1 ) 
    U_seg = U_ode(1:length(U_ode),:);
    traj = forwardIntegrateControlInput([U_seg ;U_test(end,:)],update_states);
    initial_states = traj(end,:)';
end 

if(toc>50)
    1
    break;
end

end

U_steer = x(u_start:2:end);
U_force = x(u_start+1:2:end);
U_delta = interp1(0:delta_T:time-delta_T,[U_steer U_force],0:0.01:time-delta_T);

U_test = U_delta + U_all(1+shift*(n)*delta_T*100:size(U_delta,1)+shift*(n)*delta_T*100,:);
U_ode = [U_ode;U_test(1:end,:)];

U_ode = [U_ode;U_test(1:shift*delta_T*100+1,:)];


%% plot track and obstacles
% plot decision variable
%{
traj_test = forwardIntegrateControlInput(U_ode,[287;5;-176;0;2;0]);
% reshaped 
figure(1)
hold on 
plot(traj_test(1:end,1),traj_test(1:end,3),'r')
%plot(traj_all(1:end,1),traj_all(1:end,3),'b')
plot(bl_new(1,1:end),bl_new(2,1:end),'k')
plot(br_new(1,1:end),br_new(2,1:end),'k')
plot(cline_new(1,1:end),cline_new(2,1:end),'k--')
for i=1:length(Xobs)
    plot(Xobs{i}(:,1),Xobs{i}(:,2))
end
axis equal
%}

% toc
end

function [Y]=forward(U,x)

%constants
W=13720;
Nw=2;
f=0.01;
Iz=2667;
a=1.35;
b=1.45;
By=0.27;
Cy=1.2;
Dy=2921;
Ey=-1.6;
Shy=0;
Svy=0;
m=1400;

%generate input functions
d_f=U(1);
F_x=U(2);

%slip angle functions in degrees
a_f= rad2deg(d_f-atan2(x(4)+a*x(6),x(2)));
a_r= rad2deg(-atan2((x(4)-b*x(6)),x(2)));

%Nonlinear Tire Dynamics
phi_yf= (1-Ey)*(a_f+Shy)+(Ey/By)*atan(By*(a_f+Shy));
phi_yr= (1-Ey)*(a_r+Shy)+(Ey/By)*atan(By*(a_r+Shy));


% Generate lateral forces
F_yf= Dy*sin(Cy*atan(By*phi_yf))+Svy;
F_yr= Dy*sin(Cy*atan(By*phi_yr))+Svy;

%vehicle dynamics
df= [x(2)*cos(x(5))-x(4)*sin(x(5));...
          (-f*W+Nw*F_x-F_yf*sin(d_f))/m+x(4)*x(6);...
          x(2)*sin(x(5))+x(4)*cos(x(5));...
          (F_yf*cos(d_f)+F_yr)/m-x(2)*x(6);...
          x(6);...
          (F_yf*a*cos(d_f)-F_yr*b)/Iz];
      
%Solve for trajectory      
Y = df' * 0.01 + x;

end


function y= hao( a, b)
  xx=a: (b-a)/10: b;
  y=[xx';b*ones(39,1)];
end

function y= jun ( a, b, n)
  xx=a: (b-a)/10: b;
  y=[xx';b*ones(n-1,1)];
end

function Jdf = Jdf(d_f,F_x,x)

x1=x(1);
x2=x(2);
x3=x(3);
x4=x(4);
x5=x(5);
x6=x(6);

%JDF2
%    JDF = JDF2(D_F,X2,X4,X5,X6)

%    This function was generated by the Symbolic Math Toolbox version 7.2.
%    30-Nov-2017 21:12:22

Jdf = reshape([0.0,sin(atan(d_f.*(-4.022163721818379e1)+atan(d_f.*1.546986046853223e1-angle(x2+x4.*1i+x6.*1.35i).*1.546986046853223e1).*(8.0./5.0)+angle(x2+x4.*1i+x6.*1.35i).*4.022163721818379e1).*(6.0./5.0)).*cos(d_f).*2.086428571428571+(cos(atan(d_f.*(-4.022163721818379e1)+atan(d_f.*1.546986046853223e1-angle(x2+x4.*1i+x6.*1.35i).*1.546986046853223e1).*(8.0./5.0)+angle(x2+x4.*1i+x6.*1.35i).*4.022163721818379e1).*(6.0./5.0)).*sin(d_f).*(2.475177674965156e1./((d_f.*1.546986046853223e1-angle(x2+x4.*1i+x6.*1.35i).*1.546986046853223e1).^2+1.0)-4.022163721818379e1).*2.503714285714286)./((d_f.*(-4.022163721818379e1)+atan(d_f.*1.546986046853223e1-angle(x2+x4.*1i+x6.*1.35i).*1.546986046853223e1).*(8.0./5.0)+angle(x2+x4.*1i+x6.*1.35i).*4.022163721818379e1).^2+1.0),0.0,sin(atan(d_f.*(-4.022163721818379e1)+atan(d_f.*1.546986046853223e1-angle(x2+x4.*1i+x6.*1.35i).*1.546986046853223e1).*(8.0./5.0)+angle(x2+x4.*1i+x6.*1.35i).*4.022163721818379e1).*(6.0./5.0)).*sin(d_f).*2.086428571428571-(cos(atan(d_f.*(-4.022163721818379e1)+atan(d_f.*1.546986046853223e1-angle(x2+x4.*1i+x6.*1.35i).*1.546986046853223e1).*(8.0./5.0)+angle(x2+x4.*1i+x6.*1.35i).*4.022163721818379e1).*(6.0./5.0)).*cos(d_f).*(2.475177674965156e1./((d_f.*1.546986046853223e1-angle(x2+x4.*1i+x6.*1.35i).*1.546986046853223e1).^2+1.0)-4.022163721818379e1).*2.503714285714286)./((d_f.*(-4.022163721818379e1)+atan(d_f.*1.546986046853223e1-angle(x2+x4.*1i+x6.*1.35i).*1.546986046853223e1).*(8.0./5.0)+angle(x2+x4.*1i+x6.*1.35i).*4.022163721818379e1).^2+1.0),0.0,sin(atan(d_f.*(-4.022163721818379e1)+atan(d_f.*1.546986046853223e1-angle(x2+x4.*1i+x6.*1.35i).*1.546986046853223e1).*(8.0./5.0)+angle(x2+x4.*1i+x6.*1.35i).*4.022163721818379e1).*(6.0./5.0)).*sin(d_f).*(2.07e2./1.4e2)-(cos(atan(d_f.*(-4.022163721818379e1)+atan(d_f.*1.546986046853223e1-angle(x2+x4.*1i+x6.*1.35i).*1.546986046853223e1).*(8.0./5.0)+angle(x2+x4.*1i+x6.*1.35i).*4.022163721818379e1).*(6.0./5.0)).*cos(d_f).*(2.475177674965156e1./((d_f.*1.546986046853223e1-angle(x2+x4.*1i+x6.*1.35i).*1.546986046853223e1).^2+1.0)-4.022163721818379e1).*(6.21e2./3.5e2))./((d_f.*(-4.022163721818379e1)+atan(d_f.*1.546986046853223e1-angle(x2+x4.*1i+x6.*1.35i).*1.546986046853223e1).*(8.0./5.0)+angle(x2+x4.*1i+x6.*1.35i).*4.022163721818379e1).^2+1.0),0.0,1.0./7.0e2,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,cos(x5),(cos(atan(d_f.*(-4.022163721818379e1)+atan(d_f.*1.546986046853223e1-angle(x2+x4.*1i+x6.*1.35i).*1.546986046853223e1).*(8.0./5.0)+angle(x2+x4.*1i+x6.*1.35i).*4.022163721818379e1).*(6.0./5.0)).*sin(d_f).*((x4.*4.022163721818379e1+x6.*5.429921024454812e1)./((x4+x6.*(2.7e1./2.0e1)).^2+x2.^2)-(x4.*2.475177674965156e1+x6.*3.341489861202961e1)./(((x4+x6.*(2.7e1./2.0e1)).^2+x2.^2).*((d_f.*1.546986046853223e1-angle(x2+x4.*1i+x6.*1.35i).*1.546986046853223e1).^2+1.0))).*(-2.503714285714286))./((d_f.*(-4.022163721818379e1)+atan(d_f.*1.546986046853223e1-angle(x2+x4.*1i+x6.*1.35i).*1.546986046853223e1).*(8.0./5.0)+angle(x2+x4.*1i+x6.*1.35i).*4.022163721818379e1).^2+1.0),sin(x5),-x6+(cos(atan(atan(angle(x2+x4.*1i-x6.*1.45i).*1.546986046853223e1).*(8.0./5.0)-angle(x2+x4.*1i-x6.*1.45i).*4.022163721818379e1).*(6.0./5.0)).*((x4.*4.022163721818379e1-x6.*5.83213739663665e1)./((x4-x6.*(2.9e1./2.0e1)).^2+x2.^2)-(x4.*2.475177674965156e1-x6.*3.589007628699477e1)./((angle(x2+x4.*1i-x6.*1.45i).^2.*2.393165829158561e2+1.0).*((x4-x6.*(2.9e1./2.0e1)).^2+x2.^2))).*2.503714285714286)./((atan(angle(x2+x4.*1i-x6.*1.45i).*1.546986046853223e1).*(8.0./5.0)-angle(x2+x4.*1i-x6.*1.45i).*4.022163721818379e1).^2+1.0)+(cos(atan(d_f.*(-4.022163721818379e1)+atan(d_f.*1.546986046853223e1-angle(x2+x4.*1i+x6.*1.35i).*1.546986046853223e1).*(8.0./5.0)+angle(x2+x4.*1i+x6.*1.35i).*4.022163721818379e1).*(6.0./5.0)).*cos(d_f).*((x4.*4.022163721818379e1+x6.*5.429921024454812e1)./((x4+x6.*(2.7e1./2.0e1)).^2+x2.^2)-(x4.*2.475177674965156e1+x6.*3.341489861202961e1)./(((x4+x6.*(2.7e1./2.0e1)).^2+x2.^2).*((d_f.*1.546986046853223e1-angle(x2+x4.*1i+x6.*1.35i).*1.546986046853223e1).^2+1.0))).*2.503714285714286)./((d_f.*(-4.022163721818379e1)+atan(d_f.*1.546986046853223e1-angle(x2+x4.*1i+x6.*1.35i).*1.546986046853223e1).*(8.0./5.0)+angle(x2+x4.*1i+x6.*1.35i).*4.022163721818379e1).^2+1.0),0.0,(cos(atan(atan(angle(x2+x4.*1i-x6.*1.45i).*1.546986046853223e1).*(8.0./5.0)-angle(x2+x4.*1i-x6.*1.45i).*4.022163721818379e1).*(6.0./5.0)).*((x4.*4.022163721818379e1-x6.*5.83213739663665e1)./((x4-x6.*(2.9e1./2.0e1)).^2+x2.^2)-(x4.*2.475177674965156e1-x6.*3.589007628699477e1)./((angle(x2+x4.*1i-x6.*1.45i).^2.*2.393165829158561e2+1.0).*((x4-x6.*(2.9e1./2.0e1)).^2+x2.^2))).*(-6.67e2./3.5e2))./((atan(angle(x2+x4.*1i-x6.*1.45i).*1.546986046853223e1).*(8.0./5.0)-angle(x2+x4.*1i-x6.*1.45i).*4.022163721818379e1).^2+1.0)+(cos(atan(d_f.*(-4.022163721818379e1)+atan(d_f.*1.546986046853223e1-angle(x2+x4.*1i+x6.*1.35i).*1.546986046853223e1).*(8.0./5.0)+angle(x2+x4.*1i+x6.*1.35i).*4.022163721818379e1).*(6.0./5.0)).*cos(d_f).*((x4.*4.022163721818379e1+x6.*5.429921024454812e1)./((x4+x6.*(2.7e1./2.0e1)).^2+x2.^2)-(x4.*2.475177674965156e1+x6.*3.341489861202961e1)./(((x4+x6.*(2.7e1./2.0e1)).^2+x2.^2).*((d_f.*1.546986046853223e1-angle(x2+x4.*1i+x6.*1.35i).*1.546986046853223e1).^2+1.0))).*(6.21e2./3.5e2))./((d_f.*(-4.022163721818379e1)+atan(d_f.*1.546986046853223e1-angle(x2+x4.*1i+x6.*1.35i).*1.546986046853223e1).*(8.0./5.0)+angle(x2+x4.*1i+x6.*1.35i).*4.022163721818379e1).^2+1.0),0.0,0.0,0.0,0.0,0.0,0.0,-sin(x5),x6+(cos(atan(d_f.*(-4.022163721818379e1)+atan(d_f.*1.546986046853223e1-angle(x2+x4.*1i+x6.*1.35i).*1.546986046853223e1).*(8.0./5.0)+angle(x2+x4.*1i+x6.*1.35i).*4.022163721818379e1).*(6.0./5.0)).*sin(d_f).*((x2.*4.022163721818379e1)./((x4+x6.*(2.7e1./2.0e1)).^2+x2.^2)-(x2.*2.475177674965156e1)./(((x4+x6.*(2.7e1./2.0e1)).^2+x2.^2).*((d_f.*1.546986046853223e1-angle(x2+x4.*1i+x6.*1.35i).*1.546986046853223e1).^2+1.0))).*2.503714285714286)./((d_f.*(-4.022163721818379e1)+atan(d_f.*1.546986046853223e1-angle(x2+x4.*1i+x6.*1.35i).*1.546986046853223e1).*(8.0./5.0)+angle(x2+x4.*1i+x6.*1.35i).*4.022163721818379e1).^2+1.0),cos(x5),(cos(atan(atan(angle(x2+x4.*1i-x6.*1.45i).*1.546986046853223e1).*(8.0./5.0)-angle(x2+x4.*1i-x6.*1.45i).*4.022163721818379e1).*(6.0./5.0)).*((x2.*4.022163721818379e1)./((x4-x6.*(2.9e1./2.0e1)).^2+x2.^2)-(x2.*2.475177674965156e1)./((angle(x2+x4.*1i-x6.*1.45i).^2.*2.393165829158561e2+1.0).*((x4-x6.*(2.9e1./2.0e1)).^2+x2.^2))).*(-2.503714285714286))./((atan(angle(x2+x4.*1i-x6.*1.45i).*1.546986046853223e1).*(8.0./5.0)-angle(x2+x4.*1i-x6.*1.45i).*4.022163721818379e1).^2+1.0)-(cos(atan(d_f.*(-4.022163721818379e1)+atan(d_f.*1.546986046853223e1-angle(x2+x4.*1i+x6.*1.35i).*1.546986046853223e1).*(8.0./5.0)+angle(x2+x4.*1i+x6.*1.35i).*4.022163721818379e1).*(6.0./5.0)).*cos(d_f).*((x2.*4.022163721818379e1)./((x4+x6.*(2.7e1./2.0e1)).^2+x2.^2)-(x2.*2.475177674965156e1)./(((x4+x6.*(2.7e1./2.0e1)).^2+x2.^2).*((d_f.*1.546986046853223e1-angle(x2+x4.*1i+x6.*1.35i).*1.546986046853223e1).^2+1.0))).*2.503714285714286)./((d_f.*(-4.022163721818379e1)+atan(d_f.*1.546986046853223e1-angle(x2+x4.*1i+x6.*1.35i).*1.546986046853223e1).*(8.0./5.0)+angle(x2+x4.*1i+x6.*1.35i).*4.022163721818379e1).^2+1.0),0.0,(cos(atan(atan(angle(x2+x4.*1i-x6.*1.45i).*1.546986046853223e1).*(8.0./5.0)-angle(x2+x4.*1i-x6.*1.45i).*4.022163721818379e1).*(6.0./5.0)).*((x2.*4.022163721818379e1)./((x4-x6.*(2.9e1./2.0e1)).^2+x2.^2)-(x2.*2.475177674965156e1)./((angle(x2+x4.*1i-x6.*1.45i).^2.*2.393165829158561e2+1.0).*((x4-x6.*(2.9e1./2.0e1)).^2+x2.^2))).*(6.67e2./3.5e2))./((atan(angle(x2+x4.*1i-x6.*1.45i).*1.546986046853223e1).*(8.0./5.0)-angle(x2+x4.*1i-x6.*1.45i).*4.022163721818379e1).^2+1.0)-(cos(atan(d_f.*(-4.022163721818379e1)+atan(d_f.*1.546986046853223e1-angle(x2+x4.*1i+x6.*1.35i).*1.546986046853223e1).*(8.0./5.0)+angle(x2+x4.*1i+x6.*1.35i).*4.022163721818379e1).*(6.0./5.0)).*cos(d_f).*((x2.*4.022163721818379e1)./((x4+x6.*(2.7e1./2.0e1)).^2+x2.^2)-(x2.*2.475177674965156e1)./(((x4+x6.*(2.7e1./2.0e1)).^2+x2.^2).*((d_f.*1.546986046853223e1-angle(x2+x4.*1i+x6.*1.35i).*1.546986046853223e1).^2+1.0))).*(6.21e2./3.5e2))./((d_f.*(-4.022163721818379e1)+atan(d_f.*1.546986046853223e1-angle(x2+x4.*1i+x6.*1.35i).*1.546986046853223e1).*(8.0./5.0)+angle(x2+x4.*1i+x6.*1.35i).*4.022163721818379e1).^2+1.0),-x4.*cos(x5)-x2.*sin(x5),0.0,x2.*cos(x5)-x4.*sin(x5),0.0,0.0,0.0,0.0,x4+(cos(atan(d_f.*(-4.022163721818379e1)+atan(d_f.*1.546986046853223e1-angle(x2+x4.*1i+x6.*1.35i).*1.546986046853223e1).*(8.0./5.0)+angle(x2+x4.*1i+x6.*1.35i).*4.022163721818379e1).*(6.0./5.0)).*sin(d_f).*((x2.*5.429921024454812e1)./((x4+x6.*(2.7e1./2.0e1)).^2+x2.^2)-(x2.*3.341489861202961e1)./(((x4+x6.*(2.7e1./2.0e1)).^2+x2.^2).*((d_f.*1.546986046853223e1-angle(x2+x4.*1i+x6.*1.35i).*1.546986046853223e1).^2+1.0))).*2.503714285714286)./((d_f.*(-4.022163721818379e1)+atan(d_f.*1.546986046853223e1-angle(x2+x4.*1i+x6.*1.35i).*1.546986046853223e1).*(8.0./5.0)+angle(x2+x4.*1i+x6.*1.35i).*4.022163721818379e1).^2+1.0),0.0,-x2+(cos(atan(atan(angle(x2+x4.*1i-x6.*1.45i).*1.546986046853223e1).*(8.0./5.0)-angle(x2+x4.*1i-x6.*1.45i).*4.022163721818379e1).*(6.0./5.0)).*((x2.*5.83213739663665e1)./((x4-x6.*(2.9e1./2.0e1)).^2+x2.^2)-(x2.*3.589007628699477e1)./((angle(x2+x4.*1i-x6.*1.45i).^2.*2.393165829158561e2+1.0).*((x4-x6.*(2.9e1./2.0e1)).^2+x2.^2))).*2.503714285714286)./((atan(angle(x2+x4.*1i-x6.*1.45i).*1.546986046853223e1).*(8.0./5.0)-angle(x2+x4.*1i-x6.*1.45i).*4.022163721818379e1).^2+1.0)-(cos(atan(d_f.*(-4.022163721818379e1)+atan(d_f.*1.546986046853223e1-angle(x2+x4.*1i+x6.*1.35i).*1.546986046853223e1).*(8.0./5.0)+angle(x2+x4.*1i+x6.*1.35i).*4.022163721818379e1).*(6.0./5.0)).*cos(d_f).*((x2.*5.429921024454812e1)./((x4+x6.*(2.7e1./2.0e1)).^2+x2.^2)-(x2.*3.341489861202961e1)./(((x4+x6.*(2.7e1./2.0e1)).^2+x2.^2).*((d_f.*1.546986046853223e1-angle(x2+x4.*1i+x6.*1.35i).*1.546986046853223e1).^2+1.0))).*2.503714285714286)./((d_f.*(-4.022163721818379e1)+atan(d_f.*1.546986046853223e1-angle(x2+x4.*1i+x6.*1.35i).*1.546986046853223e1).*(8.0./5.0)+angle(x2+x4.*1i+x6.*1.35i).*4.022163721818379e1).^2+1.0),1.0,(cos(atan(atan(angle(x2+x4.*1i-x6.*1.45i).*1.546986046853223e1).*(8.0./5.0)-angle(x2+x4.*1i-x6.*1.45i).*4.022163721818379e1).*(6.0./5.0)).*((x2.*5.83213739663665e1)./((x4-x6.*(2.9e1./2.0e1)).^2+x2.^2)-(x2.*3.589007628699477e1)./((angle(x2+x4.*1i-x6.*1.45i).^2.*2.393165829158561e2+1.0).*((x4-x6.*(2.9e1./2.0e1)).^2+x2.^2))).*(-6.67e2./3.5e2))./((atan(angle(x2+x4.*1i-x6.*1.45i).*1.546986046853223e1).*(8.0./5.0)-angle(x2+x4.*1i-x6.*1.45i).*4.022163721818379e1).^2+1.0)-(cos(atan(d_f.*(-4.022163721818379e1)+atan(d_f.*1.546986046853223e1-angle(x2+x4.*1i+x6.*1.35i).*1.546986046853223e1).*(8.0./5.0)+angle(x2+x4.*1i+x6.*1.35i).*4.022163721818379e1).*(6.0./5.0)).*cos(d_f).*((x2.*5.429921024454812e1)./((x4+x6.*(2.7e1./2.0e1)).^2+x2.^2)-(x2.*3.341489861202961e1)./(((x4+x6.*(2.7e1./2.0e1)).^2+x2.^2).*((d_f.*1.546986046853223e1-angle(x2+x4.*1i+x6.*1.35i).*1.546986046853223e1).^2+1.0))).*(6.21e2./3.5e2))./((d_f.*(-4.022163721818379e1)+atan(d_f.*1.546986046853223e1-angle(x2+x4.*1i+x6.*1.35i).*1.546986046853223e1).*(8.0./5.0)+angle(x2+x4.*1i+x6.*1.35i).*4.022163721818379e1).^2+1.0)],[6,8]);

end

function [x,y,rhs] = boundary(lane,coord,obs_list)
    dis = lane-coord;
    norm = dis(1,:).^2 + dis(2,:).^2;
    [~,index]=sort (norm, 'ascend');
    start = min(index(1),index(2));
    bound = lane(:,start+1)-lane(:,start);
    if any(obs_list(:) == start) 
        v1 = coord - lane(start);
        v2 = coord - lane(start+1);
        if sum(v1.*bound)<0 || sum(v2.*bound)>0
        if index(1) == start
            start = start -1;
        else
            start = start + 1;
        end
        end
    end
    y = bound(1);
    x = -bound(2);
    rhs = lane(2,start)*bound(1)-lane(1,start)*bound(2)-bound(1)*coord(2)+bound(2)*coord(1);
end

function [Y]=forwardIntegrateControlInput(U,x0)
%function [Y]=forwardIntegrateControlInput(U,x0)
%
%Given a set of inputs and an initial condition, returns the vehicles
%trajectory. If no initial condition is specified the default for the track
%is used.
%
% INPUTS:
%   U           an N-by-2 vector of inputs, where the first column is the
%               steering input in radians, and the second column is the 
%               longitudinal force in Newtons.
%   
%   x0          a 1-by-6 vector of the initial state of the vehicle.
%
% OUTPUTS:
%   Y           an N-by-6 vector where each column is the trajectory of the
%               state of the vehicle
%
% Written by: Matthew Porter
% Created: 13 Nov 2017
% Modified: 16 Nov 2017

%if initial condition not given use default
if nargin<2
    x0=[287,5,-176,0,2,0];
end

%generate time vector
T=0:0.01:(size(U,1)-1)*0.01;

%constants
W=13720;
Nw=2;
f=0.01;
Iz=2667;
a=1.35;
b=1.45;
By=0.27;
Cy=1.2;
Dy=2921;
Ey=-1.6;
Shy=0;
Svy=0;
m=1400;

%generate input functions
d_f=@(t) interp1(T,U(:,1),t,'previous','extrap');
F_x=@(t) interp1(T,U(:,2),t,'previous','extrap');

%slip angle functions in degrees
a_f=@(t,x) rad2deg(d_f(t)-atan2(x(4)+a*x(6),x(2)));
a_r=@(t,x) rad2deg(-atan2((x(4)-b*x(6)),x(2)));

%Nonlinear Tire Dynamics
phi_yf=@(t,x) (1-Ey)*(a_f(t,x)+Shy)+(Ey/By)*atan(By*(a_f(t,x)+Shy));
phi_yr=@(t,x) (1-Ey)*(a_r(t,x)+Shy)+(Ey/By)*atan(By*(a_r(t,x)+Shy));


% Generate lateral forces
F_yf=@(t,x) Dy*sin(Cy*atan(By*phi_yf(t,x)))+Svy;
F_yr=@(t,x) Dy*sin(Cy*atan(By*phi_yr(t,x)))+Svy;

%vehicle dynamics
df=@(t,x) [x(2)*cos(x(5))-x(4)*sin(x(5));...
          (-f*W+Nw*F_x(t)-F_yf(t,x)*sin(d_f(t)))/m+x(4)*x(6);...
          x(2)*sin(x(5))+x(4)*cos(x(5));...
          (F_yf(t,x)*cos(d_f(t))+F_yr(t,x))/m-x(2)*x(6);...
          x(6);...
          (F_yf(t,x)*a*cos(d_f(t))-F_yr(t,x)*b)/Iz];
      
%Solve for trajectory      
[~,Y]=ode45(df,T,x0);

end

