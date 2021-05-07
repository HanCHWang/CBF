clear all
clc
%%
% % %very simple senario: f(x)=1/2*x^2; x \in [-1,1], x+2 \le 0
% % %update barrier parameter via dual variable
% lambda(1)=1;
% c(1)=1/2;
% CBF(1)=2;
% for k=1:100
%     cvx_begin
%         variable u
%         minimize norm(u)+lambda(k)*(u+CBF(k));
%         subject to
%         u<=1;
%         -u<=1;
%     cvx_end
%     lambda(k+1)=lambda(k)+c(k)*(u+CBF(k));
%     CBF(k+1)=CBF(k)-(lambda(k+1)-lambda(k));
%     c(k+1)=c(k)*9/10;
% end

%%
% % %a little more complex scenario: f(x)=1/2*x^2; x /le 1/2, x /ge -1/2
% % %constraint: u+5/8 * p /le 0

% lambda(1)=1;
% c(1)=1/2;
% CBF(1)=5/8;
% for k=1:200
%     cvx_begin
%         variable u
%         minimize norm(u)+lambda(k)*(u+CBF(k));
%         subject to
%         u<=1/2;
%         -u<=1/2;
%     cvx_end
%     lambda(k+1)=lambda(k)+c(k)*(u+CBF(k));
%     CBF(k+1)=CBF(k)-(lambda(k+1)-lambda(k));
%     c(k+1)=c(k)*9/10;
% end

%%
% % %multiple CBF for second order system
% % %constraint: u+5/8 * p /le 0

% lambda1(1)=1;
% lambda2(1)=1;
% c(1)=1/2;
% CBF1(1)=5/8;
% CBF2(1)=3;
% for k=1:200
%     cvx_begin
%         variable u
%         minimize norm(u)+lambda1(k)*(u+CBF1(k))+lambda2(k)*(-u+CBF2(k));
%         u<=1;
%         u<=-1;
%     cvx_end
%     lambda1(k+1)=lambda1(k)+c(k)*(u+CBF1(k));
%     lambda2(k+1)=lambda2(k)+c(k)*(-u+CBF2(k));
%     CBF1(k+1)=CBF1(k)-((lambda1(k+1)-lambda1(k))>0)*(lambda1(k+1)-lambda1(k));
%     CBF2(k+1)=CBF2(k)-((lambda2(k+1)-lambda2(k))>0)*(lambda2(k+1)-lambda2(k));
%     c(k+1)=c(k)*9/10;
% end
