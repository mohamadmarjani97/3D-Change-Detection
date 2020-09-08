% This Function Use Leaste Square To Fit Plane for 3D points
function [X,Y,Z,B]=PlaneFitting(x,y,z,Grid)
%% Input: x,y,z from 3D point and grid =50 for example
arr=[x,y,z];
DM = [x, y, ones(size(z))];                             % Design Matrix
B = DM\z;                                               % Estimate Parameters
[X,Y] = meshgrid(linspace(min(x),max(x),Grid), linspace(min(y),max(y),Grid));
Z = B(1)*X + B(2)*Y + B(3)*ones(size(X));
% figure;
% plot3(arr(:,1),arr(:,2),arr(:,3),'.')
% hold on
% meshc(X, Y, Z)
% hold off
% grid on
% xlabel('x(mm)'); ylabel('y(mm)'); zlabel('z(mm)');
% title('Masked plot');
% grid on
% text(-20, 50, 450, sprintf('Z = %.3f\\cdotX %+.3f\\cdotY %+3.0f', B))
end