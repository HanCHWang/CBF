clear all
clc
%%
%three agent collision avoidance with decentralized CBF




%%
%three agent collision avoidance with adaptive CBF
A=[0 1 1;1 0 1;1 1 0];%adjacent matrix
vd=1/2*[1 1;0 -1;-1 1];
velocity_range=[1 1];
acc_range=[-1 1];
timestep=0.05;
N=size(A,2);
DS=1;

robot1_position=[0;0];
robot2_position=[5;10];
robot3_position=[10;0];
robot1_velocity=[0;0];
robot2_velocity=[0;0];
robot3_velocity=[0;0];

p_12_1=1;
p_12_2=1;
p_13_1=1;
p_13_2=1;
p_23_1=1;
p_23_2=1;

for time=1:100
    %relative position
    z_12(:,time)=robot1_position(:,time)-robot2_position(:,time);
    z_13(:,time)=robot1_position(:,time)-robot3_position(:,time);
    z_23(:,time)=robot2_position(:,time)-robot3_position(:,time);
    %relative velocity
    v_12(:,time)=robot1_velocity(:,time)-robot3_velocity(:,time);
    v_13(:,time)=robot1_velocity(:,time)-robot3_velocity(:,time);
    v_23(:,time)=robot2_velocity(:,time)-robot3_velocity(:,time);
    
    %CBF constraint
    a_12=-2*z_12(:,time);
    a_21=-a_12;
    a_13=-2*z_13(:,time);
    a_31=-a_13;
    a_23=-2*z_23(:,time);
    a_32=-a_23;
    b_12=-2*v_12(:,time)'*v_12(:,time)-2*p_12_1*z_12(:,time)'*v_12(:,time)-p_12_2*(2*z_12(:,time)'*v_12(:,time)+p_12_1*(norm(z_12(:,time),2)-DS^2));
    b_21=-b_12;
    b_13=-2*v_13(:,time)'*v_13(:,time)-2*p_13_1*z_13(:,time)'*v_13(:,time)-p_13_2*(2*z_13(:,time)'*v_13(:,time)+p_13_1*(norm(z_13(:,time),2)-DS^2));
    b_31=-b_13;
    b_23=-2*v_23(:,time)'*v_23(:,time)-2*p_23_1*z_23(:,time)'*v_23(:,time)-p_23_2*(2*z_23(:,time)'*v_23(:,time)+p_23_1*(norm(z_23(:,time),2)-DS^2));
    b_32=-b_23;
    
    %CBF dual
    lambda_12(1)=1;
    lambda_21(1)=1;
    lambda_13(1)=1;
    lambda_31(1)=1;
    lambda_23(1)=1;
    lambda_32(1)=1;
    
    %CLF dual
    lambda_1(1)=1;
    lambda_2(1)=1;
    lambda_3(1)=1;
    
    %stepsize
    c(1)=1/2;
    for k=1:99
        %%
        %consensus on dual variable
        l_12(k)=1/2*(lambda_12(k)+lambda_21(k));
        l_13(k)=1/2*(lambda_13(k)+lambda_31(k));
        l_23(k)=1/2*(lambda_23(k)+lambda_32(k));
        
        %%
        %update primal decision variable
        u_1(:,k+1)=-l_12(k)*a_12-l_13(k)*a_13-lambda_1(k)*(2*robot1_velocity(:,time)-2*vd(1,:)');
        if u_1(:,k+1)>acc_range(2)
            u_1(:,k+1)=acc_range(2);
        elseif u_1(:,k+1)<acc_range(1)
            u_1(:,k+1)<acc_range(1);
            u_1(:,k+1)=acc_range(1);
        end
        
        u_2(:,k+1)=-l_12(k)*a_21-l_23(k)*a_23-lambda_2(k)*(2*robot2_velocity(:,time)-2*vd(2,:)');
        if u_2(:,k+1)>acc_range(2)
            u_2(:,k+1)=acc_range(2);
        elseif u_2(:,k+1)<acc_range(1)
            u_2(:,k+1)<acc_range(1);
            u_2(:,k+1)=acc_range(1);
        end
        
        u_3(:,k+1)=-l_13(k)*a_31-l_23(k)*a_23-lambda_3(k)*(2*robot3_velocity(:,time)-2*vd(3,:)');
        if u_3(:,k+1)>acc_range(2)
            u_3(:,k+1)=acc_range(2);
        elseif u_3(:,k+1)<acc_range(1)
            u_3(:,k+1)<acc_range(1);
            u_3(:,k+1)=acc_range(1);
        end
        
        %%
        %update CBF dual variable
        lambda_12(k+1)=max(0,l_12(k)+c(k)*(a_12'*u_1(:,k+1)+1/2*b_12));
        lambda_21(k+1)=max(0,l_12(k)+c(k)*(a_21'*u_2(:,k+1)+1/2*b_21));
        lambda_13(k+1)=max(0,l_13(k)+c(k)*(a_13'*u_1(:,k+1)+1/2*b_13));
        lambda_31(k+1)=max(0,l_13(k)+c(k)*(a_31'*u_3(:,k+1)+1/2*b_31));
        lambda_23(k+1)=max(0,l_23(k)+c(k)*(a_23'*u_2(:,k+1)+1/2*b_23));
        lambda_32(k+1)=max(0,l_23(k)+c(k)*(a_32'*u_3(:,k+1)+1/2*b_32));
        
        %update CLF dual variable
        lambda_1(k+1)=max(0,lambda_1(k)+c(k)*2*(robot1_velocity(:,time)-vd(1,:)')'*u_1(:,k+1)+0.5*norm((robot1_velocity(:,time)-vd(1,:)'),2));
        lambda_2(k+1)=max(0,lambda_2(k)+c(k)*2*(robot2_velocity(:,time)-vd(2,:)')'*u_2(:,k+1)+0.5*norm((robot2_velocity(:,time)-vd(2,:)'),2));
        lambda_3(k+1)=max(0,lambda_3(k)+c(k)*2*(robot3_velocity(:,time)-vd(3,:)')'*u_3(:,k+1)+0.5*norm((robot3_velocity(:,time)-vd(3,:)'),2));
        
        %%
        %update stepsize
        c(k+1)=c(k)*1/2;
    end
    robot1_velocity(:,time+1)=robot1_velocity(:,time)+timestep*u_1(100);
    robot1_position(:,time+1)=robot1_position(:,time)+timestep*robot1_velocity(:,time);
    robot2_velocity(:,time+1)=robot2_velocity(:,time)+timestep*u_2(100);
    robot2_position(:,time+1)=robot2_position(:,time)+timestep*robot2_velocity(:,time);
    robot3_velocity(:,time+1)=robot3_velocity(:,time)+timestep*u_3(100);
    robot3_position(:,time+1)=robot3_position(:,time)+timestep*robot3_velocity(:,time);
end
