% Inverse Kinematics of simple 3DOF Manipulator
% Desired Path
q_x = 80;
q_y = [30:0.01:90];

% Basic Variables
d_1 = 80.304;
a_2 = 128;
a_3 = 124;
a_4 = 153.466;

p_x = q_x;
p_y = q_y + a_4 - d_1;

theta_1 = [];
theta_2 = [];
theta_3 = [];

% Inverse Kinematics
for i=1:length(p_y)
    K = ((p_x)^2 + (p_y(i))^2 - (a_2)^2 - (a_3)^2)/(2*a_2*a_3);
    theta_2(i) = asin(K);
end

for i=1:length(p_y)
    cosine_1 = (p_x*(a_2+a_3*sin(theta_2(i))) - p_y(i)*a_3*cos(theta_2(i)))/((a_2+a_3*sin(theta_2(i)))^2+(a_3*cos(theta_2(i)))^2);
    sine_1 = (p_x*(a_3*cos(theta_2(i))) + p_y(i)*(a_2+a_3*sin(theta_2(i))))/((a_2+a_3*sin(theta_2(i)))^2+(a_3*cos(theta_2(i)))^2);
    theta_1(i) = atan2(sine_1,cosine_1);
end

for i=1:length(p_y)
    temp = - theta_1(i) - theta_2(i);
    theta_3(i) = temp;
end

% Forward Kinematics for testing purposes
q_x_test = [];
q_y_test = [];

%{
figure(1)
set(gcf,'units','points','position',[50,100,800,600])
for i=1:length(p_y)
    q_x_test = [0, 0, a_2*cos(theta_1(i)), a_2*cos(theta_1(i)) + a_3*cos(theta_1(i)+theta_2(i)-pi/2), ...
        a_2*cos(theta_1(i)) + a_3*cos(theta_1(i)+theta_2(i)-pi/2) + a_4*cos(theta_1(i)+theta_2(i)+theta_3(i)-pi/2)]
    q_y_test = [0, d_1, a_2*sin(theta_1(i)) + d_1, a_2*sin(theta_1(i)) + a_3*sin(theta_1(i)+theta_2(i)-pi/2) + d_1, ...
        a_2*sin(theta_1(i)) + a_3*sin(theta_1(i)+theta_2(i)-pi/2) + a_4*sin(theta_1(i)+theta_2(i)+theta_3(i)-pi/2) + d_1]
    plot(q_x_test, q_y_test)
    title(i)
    xlim([-60,150])
    ylim([-10, 300])
    pause(0.001)
end
%}

theta_1_ = 270 - (theta_1 * 180/pi);
theta_2_ = 180 - (theta_2 * 180/pi);
theta_3_ = 180 - (theta_3 * 180/pi);

    
    
    
    
    
    
    
    
    
    
    
    
    
    