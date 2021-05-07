%use control lyapunov function and control barrier function
%switch position while avoiding collision
%use CVX for solving QP


%draw obstacle as box
clear all;
clc;
obstacle_position=[3,6;3,3];
obstacle_radius=[0.5,0.5];
robot_safe_radius=[0.3,0.3];

robot_init_position=[0,8;3,3];%on the line parallel to horizontal axis

detect_range=1;

%Control Lyapunov Function V1=(x-8)^2+(y-3)^2 V2=x^2+(y-3)^2
%Control Barrier Function (p_i-p_o)^2-(R+Ds)
%first consider just avoid collision of obstacle



%%
%run one robot at first
%using control lyapunov function
% % % C=1;%coefficient of control lyapunov function
% % % k=0.3;%coefficient of controller
% % % robot1_position(:,1)=robot_init_position(:,1);
% % % flag=1;
% % % for i=2:100
% % % %     cvx_begin
% % % %     variable u(2)
% % % %     minimize norm(u);
% % % %     [(2*robot1_position(1,i-1)-16);2*(robot1_position(2,i-1))-6]'*u+C*((robot1_position(1,i-1)-8)^2+(robot1_position(2,i-1)-3)^2) <=0;
% % % %    ([(2*robot1_position(1,i-1)-6);2*robot1_position(2,i-1)-6])'*u+C*((robot1_position(1,i-1)-3)^2+(robot1_position(2,i-1)-3)^2-(robot_safe_radius(1)+obstacle_radius(1)))^3>=0;
% % % %     cvx_end
% % % if (robot1_position(1,i-1)-3)^2+(robot1_position(2,i-1)-3)^2<1.3
% % %     flag=flag+1;
% % %     cvx_begin
% % %     variable u(2)
% % %     variable delta
% % %     minimize norm(u)+norm(delta);
% % %     subject to
% % %         [(2*robot1_position(1,i-1)-16);2*(robot1_position(2,i-1))-6]'*u+C*((robot1_position(1,i-1)-8)^2+(robot1_position(2,i-1)-3)^2) <=0;
% % %         ([2*robot1_position(1,i-1)-6;2*robot1_position(2,i-1)-6])'*u+C*((robot1_position(1,i-1)-3)^2+(robot1_position(2,i-1)-3)^2-(robot_safe_radius(1)+obstacle_radius(1)))^3+delta>=0;
% % %     cvx_end
% % % else
% % %     cvx_begin
% % %     variable u(2)
% % %     variable delta
% % %     minimize norm(u)+norm(delta);
% % %     subject to
% % %         [(2*robot1_position(1,i-1)-16);2*(robot1_position(2,i-1))-6]'*u+C*((robot1_position(1,i-1)-8)^2+(robot1_position(2,i-1)-3)^2) <=0;
% % % %         ([2*robot1_position(1,i-1)-6;2*robot1_position(2,i-1)-6])'*u+C*((robot1_position(1,i-1)-3)^2+(robot1_position(2,i-1)-3)^2-(robot_safe_radius(1)+obstacle_radius(1)))^3+delta>=0;
% % %     cvx_end
% % % end
% % %     
% % %     robot1_position(:,i)=robot1_position(:,i-1)+k*u;
% % %     
% % % end

%%
%consider only one robot
%only use control barrier function
% % % C=1;%coefficient of control barrier function
% % % k=0.15;%coefficient of controller
% % % robot1_position(:,1)=robot_init_position(:,1);
% % % for i=2:400
% % %     u_nom=0.05*[8-robot1_position(1,i-1);3-robot1_position(2,i-1)];
% % %     if (robot1_position(1,i-1)-obstacle_position(1,1))^2+(robot1_position(2,i-1)-obstacle_position(2,1))^2<1&& (robot1_position(1,i-1)-obstacle_position(1,2))^2+(robot1_position(2,i-1)-obstacle_position(2,2))^2>1
% % %         flag=flag+1;
% % %         cvx_begin
% % %         variable u(2)
% % %         variable delta
% % %         minimize norm(u-u_nom)+norm(delta);
% % %         subject to
% % %         ([2*robot1_position(1,i-1)-6;2*robot1_position(2,i-1)-6])'*u+C*((robot1_position(1,i-1)-3)^2+(robot1_position(2,i-1)-3)^2-(robot_safe_radius(1)+obstacle_radius(1)))^3+delta>=0;
% % %         u<=0.3;
% % %         cvx_end
% % %     elseif (robot1_position(1,i-1)-obstacle_position(1,1))^2+(robot1_position(2,i-1)-obstacle_position(2,1))^2<1&& (robot1_position(1,i-1)-obstacle_position(1,2))^2+(robot1_position(2,i-1)-obstacle_position(2,2))^2<1
% % %         cvx_begin
% % %         variable u(2)
% % %         variable delta
% % %         minimize norm(u-u_nom)+norm(delta);
% % %         subject to
% % %             ([2*robot1_position(1,i-1)-6;2*robot1_position(2,i-1)-6])'*u+C*((robot1_position(1,i-1)-3)^2+(robot1_position(2,i-1)-3)^2-(robot_safe_radius(1)+obstacle_radius(1)))^3+delta>=0;
% % %             ([2*robot1_position(1,i-1)-12;2*robot1_position(2,i-1)-6])'*u+C*((robot1_position(1,i-1)-6)^2+(robot1_position(2,i-1)-3)^2-(robot_safe_radius(1)+obstacle_radius(1)))^3+delta>=0;
% % %             u<=0.3;
% % %         cvx_end
% % %     elseif (robot1_position(1,i-1)-obstacle_position(1,1))^2+(robot1_position(2,i-1)-obstacle_position(2,1))^2>1&& (robot1_position(1,i-1)-obstacle_position(1,2))^2+(robot1_position(2,i-1)-obstacle_position(2,2))^2<1
% % %         cvx_begin
% % %         variable u(2)
% % %         variable delta
% % %         minimize norm(u-u_nom)+norm(delta);
% % %         subject to
% % %             ([2*robot1_position(1,i-1)-12;2*robot1_position(2,i-1)-6])'*u+C*((robot1_position(1,i-1)-6)^2+(robot1_position(2,i-1)-3)^2-(robot_safe_radius(1)+obstacle_radius(1)))^3+delta>=0;
% % %             u<=0.3;
% % %         cvx_end
% % %     else
% % %         u=u_nom;
% % %     end
% % %     
% % %     robot1_position(:,i)=robot1_position(:,i-1)+k*u;
% % %     
% % % end

%%
%two hop collision avoidance
C=1;%coefficient of control barrier function
k=0.5;%coefficient of controller
robot1_position(:,1)=robot_init_position(:,1);
flag=1;
distance=0;
for i=2:400
    u_nom=0.05*[8-robot1_position(1,i-1);3-robot1_position(2,i-1)];
    if (robot1_position(1,i-1)-obstacle_position(1,1))^2+(robot1_position(2,i-1)-obstacle_position(2,1))^2<detect_range&& (robot1_position(1,i-1)-obstacle_position(1,2))^2+(robot1_position(2,i-1)-obstacle_position(2,2))^2>detect_range
        flag=flag+1;
        cvx_begin
        variable u(2)
        variable delta
        minimize norm(u-u_nom);
        subject to 
        ([2*robot1_position(1,i-1)-6;2*robot1_position(2,i-1)-6])'*u+C*((robot1_position(1,i-1)-3)^2+(robot1_position(2,i-1)-3)^2-(robot_safe_radius(1)+obstacle_radius(1)))>=0;
        ([2*robot1_position(1,i-1)-12;2*robot1_position(2,i-1)-6])'*u+0.1*C*((robot1_position(1,i-1)-6)^2+(robot1_position(2,i-1)-3)^2-(robot_safe_radius(1)+obstacle_radius(1)))>=0;
        u<=0.3;
        cvx_end
    elseif (robot1_position(1,i-1)-obstacle_position(1,1))^2+(robot1_position(2,i-1)-obstacle_position(2,1))^2<detect_range&& (robot1_position(1,i-1)-obstacle_position(1,2))^2+(robot1_position(2,i-1)-obstacle_position(2,2))^2<detect_range
        flag=flag+1;
        cvx_begin
        variable u(2)
        variable delta
        minimize norm(u-u_nom);
        subject to
            ([2*robot1_position(1,i-1)-6;2*robot1_position(2,i-1)-6])'*u+C*((robot1_position(1,i-1)-3)^2+(robot1_position(2,i-1)-3)^2-(robot_safe_radius(1)+obstacle_radius(1)))>=0;
            ([2*robot1_position(1,i-1)-12;2*robot1_position(2,i-1)-6])'*u+C*((robot1_position(1,i-1)-6)^2+(robot1_position(2,i-1)-3)^2-(robot_safe_radius(1)+obstacle_radius(1)))>=0;
            u<=0.3;
        cvx_end
    elseif (robot1_position(1,i-1)-obstacle_position(1,1))^2+(robot1_position(2,i-1)-obstacle_position(2,1))^2>detect_range&& (robot1_position(1,i-1)-obstacle_position(1,2))^2+(robot1_position(2,i-1)-obstacle_position(2,2))^2<detect_range
        cvx_begin
        variable u(2)
        variable delta
        minimize norm(u-u_nom);
        subject to
            ([2*robot1_position(1,i-1)-12;2*robot1_position(2,i-1)-6])'*u+C*((robot1_position(1,i-1)-6)^2+(robot1_position(2,i-1)-3)^2-(robot_safe_radius(1)+obstacle_radius(1)))>=0;
            u<=0.3;
        cvx_end
    else
        cvx_begin
        variable u(2)
        variable delta
        minimize norm(u-u_nom)
        subject to
       ([2*robot1_position(1,i-1)-12;2*robot1_position(2,i-1)-6])'*u+0.3*C*((robot1_position(1,i-1)-6)^2+(robot1_position(2,i-1)-3)^2-(robot_safe_radius(1)+obstacle_radius(1)))>=0;
            u<=0.3;
        cvx_end
%         u=u_nom;
    end
    
    robot1_position(:,i)=robot1_position(:,i-1)+k*u;
    distance=distance+norm(robot1_position(:,i)-robot1_position(:,i-1),1);
    
end


%%
%plot the collision avoidance
figure 
% plot(robot1_position(1,:),robot1_position(2,:),'*');
for i=1:400
%     viscircles([robot1_position(1,i),robot1_position(2,i)],0.3,'Color','b');
    plot(robot1_position(1,:),robot1_position(2,:),'*');
    hold on
end
viscircles(obstacle_position(:,1)',0.8);
viscircles(obstacle_position(:,2)',0.8);
% viscircles(obstacle_position(:,2),0.5);
% set(gca,'DataAspectRatio',[1 1 1])
hold off



