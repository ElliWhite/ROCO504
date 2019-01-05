clc;
close all;

x = 10;
l = 0.2;                    %Body - Distance of C.o.M from pivot point
ThetaMax = 70;              %Failure angle
L1 = linspace(0.1,1,x);     %Leg - Distance of Leg Pivot from Body pivot
L2 = linspace(0.1,1,x);     %Leg - Distance of C.o.M from pivot point
L3 = linspace(0.1,1,x);     %Leg - Length
m = linspace(0.1,1,x);      %Leg - Mass


for i = 1:x
    for j = 1:x
        for k = 1:x
            for h = 1:x
                Ta = ActivationWindow(l, ThetaMax);
                T  = LegTorque( m(i), L1(j), L2(k), L3(h), Ta );
                T = T.';
                t = [l, L1(j), L2(k), L3(h), m(i), ThetaMax, T];
                dlmwrite('torque.csv',t,'delimiter',',', '-append');
            end
        end
    end
end