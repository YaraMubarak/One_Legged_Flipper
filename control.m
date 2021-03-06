function result = control1()
% control vertical hopper

global h_axes body leg
global dt time x y xd yd 
global hip_torque leg_angle body_angle leg_angled body_angled
global leg_state foot_x foot_y leg_lengthd leg_length rest_leg_length
global control_state height_desired leg_angle_desired last_bounce_time
global last_touchdown_time last_takeoff_time max_height last_max_height
global speed_desired
%Michael's Values
global integration 

global speed_integral

global bitbit
global pancake_flag pancake_indic  

global masterSpeed
global currL
global LcontrolCounter
global Ldesired
global pastX
global tempL
global centroid
global centroidIntegral

g = 9.81;




% control_state values
init = 0;
in_air = 1;
on_ground_going_down = 2;
on_ground_going_up = 3;

%Michael's Modified Control
% hip_air_k = 10;
hip_air_k = 500;
% hip_air_b = 1;
hip_air_b = 20;
% hip_grnd_k = 10;
hip_grnd_k = 300;
% hip_grnd_b = 1;
hip_grnd_b = 20;

%Variables for speed control

kx_speed = 0.0007;
kx_speed_I = 0.000005;

if pancake_flag 
     kx = 6e-3; 
end 


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
  
  centroid = [centroid;
              x];
  
  if ~pancake_indic
      pancake_indic = true;
  end
 
%   %Speed control by modulating the desired leg angle
%   if and(pancake_flag,pancake_indic)  
%       speed_desired = -speed_desired ; 
%       pancake_indic = not(pancake_indic) ; 
%   elseif and(pancake_flag,not(pancake_indic) ) 
%         pancake_indic = not(pancake_indic) ; 
%   end 
  
  Ts = last_bounce_time;
  speed_integral = speed_integral + xd - speed_desired;
  
  if not(pancake_flag) 
  %Want to use the equation given in Raibert control
    xf = Ts*xd/2 + kx_speed*(xd - speed_desired) + kx_speed_I*speed_integral;
    leg_angle_desired = (asin(xf/leg_length)) - 2*pi;
  else 
  %Yara's controller
    xf = xd*Ts/2  +kx*(abs(xd) - 4.25*speed_desired) ;
    leg_angle_desired = asin((xf)/leg_length);
  end 
  
  if not(bitbit)
      leg_angle_desired = 0;
  end


  hip_torque = hip_air_k*(-leg_angle_desired + leg_angle) + ...
                             hip_air_b*(leg_angled);
  if ( y > max_height )
    max_height = y;
  end;
  if ( yd < 0 )
    last_max_height = max_height;
  end;
%    bitbit = true;
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
  hip_torque = hip_grnd_k*(0 -body_angle) + ...
      hip_grnd_b*(0 - body_angled);
  
  
  
  %%%This is where the BIG BOI CODE IS%%%%%%
  
  if pancake_indic
  
      currL = x - pastX;
      pastX = x;
      
      centroidIntegral = centroidIntegral + mean(centroid)
      
      if sign(speed_desired) == -1 
          speed_desired = -(speed_desired)
      else
          speed_desired = -speed_desired - 3*mean(centroid) - 0.01*centroidIntegral
      end

      
      if LcontrolCounter == 0
          LcontrolCounter = LcontrolCounter + 1;
      elseif LcontrolCounter == 1
          LcontrolCounter = 0;
%           speed_desired = sign(speed_desired)*(abs(speed_desired) + ...
%                     0.01*(masterSpeed - abs(xd)));
          
          tempL = tempL + (Ldesired - abs(currL))*0.1;
          testheight = ((tempL*g/(2*speed_desired))^2) / (2*g);            
          testSpeed = tempL*g / (2*sqrt(2*g*height_desired));
          
%           height_desired = height_desired + 0.01*(testheight-max_height)
    
%           height_desired = height_desired + 0.02*(testheight - max_height)
          speed_desired = testSpeed;
          centroid = [];
      end     
     
      pancake_indic = false;
  end
  
   
 
  
end;

if control_state == on_ground_going_up
  % SET rest_leg_length TO ADD ENERGY
%   rest_leg_length = leg_length_default;
  %Michael's controller code
  bitbit = true;
  
  
  
  
 
 %Original Heigh Controller
  error = height_desired - max_height; 
  integration = integration + error;
  rest_leg_length = rest_leg_length + 0.25*(error) + 0.001*integration;
    

  
 
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
%   hip_torque = hip_grnd_k*(-body_angle) - hip_grnd_b*body_angled;
  
  hip_torque = hip_grnd_k*(0 -body_angle) + ...
      hip_grnd_b*(0 - body_angled);

  
end;



  
%   if bitbit
%       leg_angle_desired = leg_angle_desired + pi*2;
%        bitbit = false; %set flag to be false so it doesnt add more 
%   end

