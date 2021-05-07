%saving safety CBF
clear all
clc
%%


time=300;
robot_position(:,1)=[0;1];
obstacle_position=[5;0];
target_position=[10;0];
timestep=0.03;
PID_P=0.1;%using P controller
DS=1;%safe distance
gamma=5;%CBF parameter

%initialize safety saving
acc(1)=0;

for i=1:time
    u_nom(:,i)=PID_P*[target_position(1)-robot_position(1,i);target_position(2)-robot_position(2,i)];
    cvx_begin
        variable u(2)
        variable delta
%         minimize 1/2*10*power(2,norm(u-u_nom(:,i)))-1/2*delta;
        minimize 1/2*10*power(2,norm(u-u_nom(:,i)));
        subject to
        u<=1;
        -u<=1;
        delta>=0;
%         [2*robot_position(1,i)-10;2*robot_position(2,i)]'*u+gamma*((robot_position(1,i)-5)^2+robot_position(2,i)^2-1)+acc(i)>=delta;
    [2*robot_position(1,i)-10;2*robot_position(2,i)]'*u+gamma*((robot_position(1,i)-5)^2+robot_position(2,i)^2-1)>=0;
    cvx_end
    robot_position(:,i+1)=robot_position(:,i)+timestep*u;
%     acc(i+1)=(acc(i)+([2*robot_position(1,i)-10;2*robot_position(2,i)]'*u+gamma*((robot_position(1,i)-5)^2+(robot_position(2,i)^2-1))));
    acc(i+1)=1/(1+timestep*gamma)*acc(i)+([2*robot_position(1,i)-10;2*robot_position(2,i)]'*u+gamma*((robot_position(1,i)-5)^2+(robot_position(2,i)^2-1)));
    h(i)=(robot_position(1,i)-5)^2+robot_position(2,i)^2-1;
    hdot1(i)= [2*robot_position(1,i)-10;2*robot_position(2,i)]'*u;
    hh(i)=hdot1(i)+gamma*h(i);
end