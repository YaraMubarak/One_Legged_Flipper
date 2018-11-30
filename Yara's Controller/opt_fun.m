function [cost] = opt_fun(meu) 
global ceq c 
global L_desired
global delta_x 

%meu(1) = dx_desired ;
%meu(2) = height_desired;
%meu(3) = slack_variable for desired_length 
[dt, time, x ,y, xd, yd , hip_torque, leg_angle, body_angle, ...
    leg_angled, body_angled, leg_state, foot_x , foot_y , leg_length, leg_lengthd,...
    rest_leg_length, control_state, height_desired, leg_angle_desired, last_bounce_time, ... 
    last_touchdown_time, last_takeoff_time, max_height, last_max_height, speed_desired, ytakeoff, jump_counter] = loadglobal() ; 
height_desired = meu(2) ; 
speed_desired = meu(1) ; 
prev_state = 1 ; 
jump_counter = 0 ; 

try 
while jump_counter <= 2 
    %simulate 
       [dt, time, x ,y, xd, yd , hip_torque, leg_angle, body_angle, ...
    leg_angled, body_angled, leg_state, foot_x , foot_y , leg_length, leg_lengthd,...
    rest_leg_length, control_state, height_desired, leg_angle_desired, last_bounce_time, ... 
    last_touchdown_time, last_takeoff_time, max_height, last_max_height, speed_desired, ytakeoff] =...
    simulate4opt(dt, time, x ,y, xd, yd , hip_torque, leg_angle, body_angle, ...
    leg_angled, body_angled, leg_state, foot_x , foot_y , leg_length, leg_lengthd,...
    rest_leg_length, control_state, height_desired, leg_angle_desired, last_bounce_time, ... 
    last_touchdown_time, last_takeoff_time, max_height, last_max_height, speed_desired, ytakeoff);
    
  %control 
[dt, time, x ,y, xd, yd , hip_torque, leg_angle, body_angle, ...
    leg_angled, body_angled, leg_state, foot_x , foot_y , leg_length, leg_lengthd,...
    rest_leg_length, control_state, height_desired, leg_angle_desired, last_bounce_time, ... 
    last_touchdown_time, last_takeoff_time, max_height, last_max_height, speed_desired, ytakeoff] =...
    control4opt(dt, time, x ,y, xd, yd , hip_torque, leg_angle, body_angle, ...
    leg_angled, body_angled, leg_state, foot_x , foot_y , leg_length, leg_lengthd,...
    rest_leg_length, control_state, height_desired, leg_angle_desired, last_bounce_time, ... 
    last_touchdown_time, last_takeoff_time, max_height, last_max_height, speed_desired, ytakeoff);


[jump_end] = check_jump(control_state , prev_state) ; 
prev_state = control_state ; 
if jump_end 
    jump_counter = jump_counter + 1 ; 
end 
if jump_counter == 1 
    x_initial = x ; 
end 
%cost = abs( hip_torque ); 
%cost = abs(meu(3)); 
 
end
delta_x = x - x_initial ;
cost = abs(L_desired - (delta_x)) ; 

catch
    disp('cannot simulate') 
    cost = 1e15  ; 
end 
ceq = 0 ; 
c = [];  
delta_x = x - x_initial ;
%ceq(1) = L_desired - (delta_x ) ; %+ meu(3)) ;
%ceq(2) = meu(3) ; 
end 




