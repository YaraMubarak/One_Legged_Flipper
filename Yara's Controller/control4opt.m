function[ dt, time, x ,y, xd, yd , hip_torque, leg_angle, body_angle, ...
    leg_angled, body_angled, leg_state, foot_x , foot_y , leg_length, leg_lengthd,...
    rest_leg_length, control_state, height_desired, leg_angle_desired, last_bounce_time, ... 
    last_touchdown_time, last_takeoff_time, max_height, last_max_height, speed_desired, ytakeoff] = control4opt(dt, time, x ,y, xd, yd , hip_torque, leg_angle, body_angle, ...
    leg_angled, body_angled, leg_state, foot_x , foot_y , leg_length, leg_lengthd,...
    rest_leg_length, control_state, height_desired, leg_angle_desired, last_bounce_time, ... 
    last_touchdown_time, last_takeoff_time, max_height, last_max_height, speed_desired, ytakeoff )
% control_state values
init = 0;
in_air = 1;
on_ground_going_down = 2;
on_ground_going_up = 3;

hip_air_k = 1e5;
hip_air_b = 85;
hip_grnd_k = 10;
hip_grnd_b = 1;

leg_length_default = 0.5;

leg_length_gain = 0.0;

rest_leg_length = leg_length_default;
hip_torque = 0;   

foot_y_new = y - rest_leg_length*cos( leg_angle );
leg_length_new = sqrt( (x - foot_x)^2 + (y - foot_y)^2 );

% initialization
if control_state == init
  control_state = in_air;
  result = control_state;
  return;
end;

if control_state == in_air
  if foot_y_new < 0
    last_touchdown_time = time;
    if yd <= 0
      control_state = on_ground_going_down;
    else
      control_state = on_ground_going_up;
    end;
    result = control_state;
    return;
  end;
  
  Ts = 0.25 ; 
  kx = 8e-3; 
  xf = xd*Ts/2  +kx*(xd - 4.25*speed_desired) ;
%   
  leg_angle_desired = asin((xf)/leg_length) ;
  leg_angle_desired = leg_angle_desired + 2*pi; 
  hip_torque = hip_air_k*(leg_angle - leg_angle_desired) + ...
                             hip_air_b*leg_angled;
  if ( y > max_height )
    max_height = y;
  end;
  if ( yd < 0 )
    last_max_height = max_height;
   ytakeoff = y ;
  end;
end;

if control_state == on_ground_going_down
  
  if leg_length_new > rest_leg_length
    control_state = in_air;
    max_height = y;
    result = control_state;
    last_takeoff_time = time;
    return;
  end;
  if yd > 0
    control_state = on_ground_going_up;
    result = control_state;
    return;
  end;
  hip_torque = hip_grnd_k*(-body_angle) - hip_grnd_b*body_angled;
end;

if control_state == on_ground_going_up
 weight_hyp_spring_ratio = 1e-3; 
  delta_h = height_desired -last_max_height;
  yd_des = sqrt(2*9.8*(height_desired-ytakeoff)); 
  penalty = 1e-1; 
  rest_leg_length = leg_length_default +penalty*( yd_des- yd) + sign(delta_h)*sqrt(2*weight_hyp_spring_ratio*abs(delta_h));

  if leg_length_new > rest_leg_length
    control_state = in_air;
    max_height = y;
    result = control_state;
    last_takeoff_time = time;
    if ( last_touchdown_time > 0 )
      last_bounce_time = last_takeoff_time - last_touchdown_time;
    end;
    return;
  end;
  if yd < 0
    control_state = on_ground_going_down;
    result = control_state;
    return;
  end;
  hip_torque = hip_grnd_k*(-body_angle) - hip_grnd_b*body_angled;
  
  
end;


