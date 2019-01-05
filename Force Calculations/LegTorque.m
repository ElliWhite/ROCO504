function [ T ] = LegTorque( m, L1, L2, L3, t )
x = 91;
min = 20;
max = 69;
theta = linspace(0, 90, x).';
theta1 = zeros(x,1);
theta2 = zeros(x,1);
tau = zeros(x,1);

for i = 1:x
    theta1(i) = abs(theta(i) - 90 - asind((L1 * sind(90 - theta(i))) / L2));
    if (i <= max) && (i >= min);
        theta2(i) = theta1(i) / (t(i) ^2);    
    end
    tau(i) = m * (L3 ^ 2) * theta2(i);
end

T = tau;
%disp([theta theta1 theta2 tau]);

end

