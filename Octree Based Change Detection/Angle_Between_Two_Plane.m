% This Function Returns The Angle Between Two Planes
function Angle=Angle_Between_Two_Plane(X1,Y1,Z1,X2,Y2,Z2)

[NX1,NY1,NZ1]=surfnorm(X1,Y1,Z1);
NX1=mean(mean(NX1));
NY1=mean(mean(NY1));
NZ1=mean(mean(NZ1));
[NX2,NY2,NZ2]=surfnorm(X2,Y2,Z2);
NX2=mean(mean(NX2));
NY2=mean(mean(NY2));
NZ2=mean(mean(NZ2));

Angle=acos((NX1*NX2+NY1*NY2+NZ1*NZ2)/(sqrt(NX1^2+NY1^2+NZ1^2)*sqrt(NX2^2+NY2^2+NZ2^2)));
Angle=rad2deg(Angle);
end