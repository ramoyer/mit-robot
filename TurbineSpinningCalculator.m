motors = 2; %Number of motors
stall_torque = .11; %Stall torque, N.m
max_speed = 140; %Motor no load speed in rad/s
ratio = .2717; %Transmission ratio, = turbine speed/motor speed
stall_current = 4.32; %Stall current of one motor at given voltage
voltage = 7.4; %Input voltage at conditions listed
power_dissipated = 0; %Power (heat) dissipated in motor coils
final_time = 10; %Time to calculate turbine response over, in seconds
iterations = 1000; %Number of iterations
time_taken = 0;
flag = 0;

I_wheel = .0582; %Turbine moment of inertia, kg*m^2
friction_torque = .09; % Angular friction on turbine
speed_vector = zeros(1,iterations+1); %Vector of turbine speeds
applied_torque = 0; %Applied torque, calculated in loop each iteration
delta_t = final_time/iterations;%Timestep for equation calculation


for i = 1:iterations
    
    %Applied torque: Torque = number of motors*1/ratio*(motor stall torque-
    %motor stall torque*(current motor speed/no-load motor speed)).
    %Current motor speed = speed_vector(1)/ratio
    applied_torque = motors/ratio*...
        stall_torque*(1-speed_vector(i)/ratio/max_speed)-friction_torque;
    if i < 4
        display(applied_torque);
    end
    speed_vector(i+1) = speed_vector(i) + ...
        applied_torque*delta_t/I_wheel;
    
    if flag == 0
        if speed_vector(i) > 25
            time_taken = i*final_time/iterations;
            flag = 1;
        end
    end
    
    %Calculate heat dissipation in motor
    power_dissipated = power_dissipated + ...
        voltage*stall_current*(1-speed_vector(i)/ratio/max_speed)*...
        final_time/iterations; %dt
    
    
end

plot(speed_vector);   
display(time_taken);




%display(power_dissipated);

