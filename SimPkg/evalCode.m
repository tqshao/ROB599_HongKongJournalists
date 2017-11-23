%% F2017 ME 599 Project
clear all;clc;close all;
%% Car information
% Car parameters already included in 'forwardIntegrateControllInput.m'
%% Track Information
load('TestTrack');
%% Start point set
s_start=0; % start from the race start and end at the same location after one lap

%%%% If you want to start and end somewhere else in the middle  %%%%%%%%%%%
%%%% (for testing perpuse) uncommon and edit the following %%%%%%%%%%%%%%%%
% start_index=11; % 1 to 597
% end_index=21; % 1 to 597
% % start at a certain location 
% s_start=Track.arc_s(start_index);
% Track.bstl=Track.bl(:,start_index);
% Track.bstr=Track.br(:,start_index);
% Track.bfl=Track.bl(:,end_index);
% Track.bfr=Track.br(:,end_index);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%
traj = struct; Car = struct;

steer_input = [zeros(300,1);-0.2*ones(300,1);-0.5*ones(100,1)];
force_input = 6000*[ones(300,1);ones(300,1);-10/6*ones(100,1)];
U = [steer_input, force_input];
[Y]=forwardIntegrateControlInput(U);

% traj = [Y(:,1),Y(:,3)];
traj.x = Y(:,1);
traj.vx = Y(:,2);
traj.y = Y(:,3);
traj.vy = Y(:,4);
traj.psi = Y(:,5);
input.steer = U(:,1);
input.force = U(:,2);
% check some violation
% checkTrajectory(traj,U)
% display velocity
formatSpec = "v_{lon} = %f \n v_{lat} = %f";
A1 = traj.vx(end);
A2 = traj.vy(end);
str1 = sprintf(formatSpec,A1,A2);


figure(1)
%%%%% plot total track %%%%%
hold on;axis equal;box on;
plot(TestTrack.bl(1,:),TestTrack.bl(2,:),'k','lineWidth',1.5);
plot(TestTrack.br(1,:),TestTrack.br(2,:),'k','lineWidth',1.5);
plot(TestTrack.cline(1,:),TestTrack.cline(2,:),'k--','lineWidth',1);

%%%%% plot trajectory of mass of center %%%%%
plot(traj.x,traj.y,'r-')

%%%%% plot car body with two tires (end of the moment) %%%%%
wheelscale = 2;
Car.Rw = 0.33;
Car.a=1.35; Car.b=1.45;
Xcar=[traj.x(end);traj.y(end);0];
tv=[cos(traj.psi(end));sin(traj.psi(end));0];
vline=[Xcar-Car.b*tv,Xcar+Car.a*tv];

pfront=Car.a*[cos(traj.psi(end));sin(traj.psi(end));0]+[traj.x(end);traj.y(end);0];
prear=-Car.b*[cos(traj.psi(end));sin(traj.psi(end));0]+[traj.x(end);traj.y(end);0];


Rotate=[cos(traj.psi(end)+input.steer(end)) -sin(traj.psi(end)+input.steer(end))  0
        sin(traj.psi(end)+input.steer(end))  cos(traj.psi(end)+input.steer(end))  0
        0             0           1];
frontwheelline=[pfront+Rotate*[-wheelscale*Car.Rw;0;0],pfront+Rotate*[wheelscale*Car.Rw;0;0]];
rearwheelline=[prear+wheelscale*Car.Rw*tv,prear-wheelscale*Car.Rw*tv];
L(1)=plot3(Xcar(1),Xcar(2),Xcar(3),'bo','MarkerSize',6);
L(2)=plot3(vline(1,:),vline(2,:),vline(3,:),'b','LineWidth',2);
L(3)=plot3(frontwheelline(1,:),frontwheelline(2,:),frontwheelline(3,:),'g','LineWidth',4);
L(4)=plot3(rearwheelline(1,:),rearwheelline(2,:),rearwheelline(3,:),'g','LineWidth',4);
xlim([traj.x(end)-20,traj.x(end)+20])
ylim([traj.y(end)-20,traj.y(end)+20])
text(traj.x(end)-10,traj.y(end)+10,{str1})


figure(2) % zoom in version of figure (1)
%%%%% plot total track %%%%%
hold on;axis equal;box on;
plot(TestTrack.bl(1,:),TestTrack.bl(2,:),'k','lineWidth',1.5);
plot(TestTrack.br(1,:),TestTrack.br(2,:),'k','lineWidth',1.5);
plot(TestTrack.cline(1,:),TestTrack.cline(2,:),'k--','lineWidth',1);
%%%%% plot trajectory of mass of center %%%%%
plot(traj.x,traj.y,'r-')

%%%%% plot car body with two tires (end of the moment) %%%%%
wheelscale = 2;
Car.Rw = 0.33;
Car.a=1.35; Car.b=1.45;
Xcar=[traj.x(end);traj.y(end);0];
tv=[cos(traj.psi(end));sin(traj.psi(end));0];
vline=[Xcar-Car.b*tv,Xcar+Car.a*tv];

pfront=Car.a*[cos(traj.psi(end));sin(traj.psi(end));0]+[traj.x(end);traj.y(end);0];
prear=-Car.b*[cos(traj.psi(end));sin(traj.psi(end));0]+[traj.x(end);traj.y(end);0];

Rotate=[cos(traj.psi(end)+input.steer(end)) -sin(traj.psi(end)+input.steer(end))  0
        sin(traj.psi(end)+input.steer(end))  cos(traj.psi(end)+input.steer(end))  0
        0             0           1];
frontwheelline=[pfront+Rotate*[-wheelscale*Car.Rw;0;0],pfront+Rotate*[wheelscale*Car.Rw;0;0]];
rearwheelline=[prear+wheelscale*Car.Rw*tv,prear-wheelscale*Car.Rw*tv];
L(1)=plot3(Xcar(1),Xcar(2),Xcar(3),'bo','MarkerSize',10);
L(2)=plot3(vline(1,:),vline(2,:),vline(3,:),'b','LineWidth',3);
L(3)=plot3(frontwheelline(1,:),frontwheelline(2,:),frontwheelline(3,:),'g','LineWidth',5);
L(4)=plot3(rearwheelline(1,:),rearwheelline(2,:),rearwheelline(3,:),'g','LineWidth',5);
% text(traj.x(end),traj.y(end),'\leftarrow sin(\pi)')
xlim([traj.x(end)-8,traj.x(end)+8])
ylim([traj.y(end)-8,traj.y(end)+8])
text(traj.x(end)-4,traj.y(end)+4,{str1})
