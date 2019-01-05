function [ Ta ] = ActivationWindow( l, ThetaMax )
x = 91;
theta = linspace(0, 90, x);
theta2 = zeros(x);
t = zeros(x);

for i = 1:x
    theta2(i) = (9.81 / l) * sind(theta(i));
    t(i) = sqrt((90 - theta(i)) / theta2(i));
end
for i = 1:x
    Ta(i,1) = t(i) - t(ThetaMax);
end
end


