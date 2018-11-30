%main optimization % simulate a Raibert hopper
close all 
clear all 

initdraw

% allocate global variables
global h_axes body leg min_x max_x
global dt time x y xd yd 
global hip_torque leg_angle body_angle leg_angled body_angled
global leg_state foot_x foot_y leg_lengthd leg_length rest_leg_length
global control_state height_desired leg_angle_desired last_bounce_time
global last_touchdown_time last_takeoff_time max_height last_max_height
global speed_desired
global L_desired 
global delta_x 
global c ceq 
figure(1) % choose right plot target

% intialize variables.
% stuff we want to control
L_desired = 0.1; 
height_desired = 1;
speed_desired = 0.1;

%define how to ramp_up L 
delta_L = 0.05 ; 
delta_L_down =0.01;
ramp_up = @(L) sign(L)*(abs(L) + delta_L) ;
ramp_down = @(L) sign(L)*(abs(L) - delta_L_down) ;

max_l_d = 0.5; 


% constants
dt = 0.001;

% initial state of robot
time = 0.0;
x = 0.0;
y = 1.0;
xd = 0.0;
yd = 0.0;
body_angle = 0;
leg_angle = 0.0;
body_angled = 0;
leg_angled = 0;
hip_torque = 0;
leg_state = 0; % simulator state: leg not on ground
% controller state
control_state = 0; 
last_bounce_time = 0.25;
last_touchdown_time = -1;
last_takeoff_time = -1;
max_height = y;
last_max_height = y;

delta_xs = [] ;
max_n_points = 3000;

% allocate array
array(max_n_points,9) = 0;
prev_state = 1 ; 
jump_counter = 0 ; 
did_you_optimize_for_this_jump = false ;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% outer loop: save data and draw picture at slow rate
for i = 1:5000


% simulate and control at faster rate
 for j = 1:10
  control2();
  simulate();
  
  
[jump_end] = check_jump(control_state , prev_state) ;
prev_state = control_state ; 
if jump_end 
    jump_counter = jump_counter + 1;
    did_you_optimize_for_this_jump = false ;
end

if not(did_you_optimize_for_this_jump) 
    options = optimoptions('fmincon','Display','iter', 'StepTolerance', 1e-10);
%     xopt = fmincon(@opt_fun , [sign(L_desired)*abs(speed_desired), height_desired, 0.1],...
%                             [],[],[],[], [-5,0.5, -abs(L_desired)*0.05],...
%                             [5,1.3, abs(L_desired)*0.05], @nonlcon,options) 
    xopt = fmincon(@opt_fun , [sign(L_desired)*abs(speed_desired), height_desired],...
                            [],[],[],[], [-3,0.5],...
                            [3,1.25], @nonlcon,options) 
    did_you_optimize_for_this_jump = not(did_you_optimize_for_this_jump) ; 
    speed_desired = xopt(1) ; 
    height_desired = xopt(2) ;
    L_desired = -L_desired; 
    delta_xs = [delta_xs , delta_x] ; 
    disp(round(abs(L_desired),2)  == round(abs(delta_x),2)  ) 
 
    if round(abs(L_desired),2)  == round(abs(delta_x),2)  && rem(jump_counter,2) ==0 && abs(L_desired) < max_l_d 
        %L_desired = ramp_up(L_desired) ;
    elseif  round(abs(L_desired),2)  ~= round(abs(delta_x),2) && rem(jump_counter,2) ==0 && abs(L_desired) > 0 
        %L_desired = ramp_down(L_desired) ;

    end 
    

end 

end

% save data in array
 if ( i <= max_n_points )
   array(i,1) = time;
   array(i,2) = y;
   array(i,3) = yd;
   array(i,4) = control_state;
   array(i,5) = body_angle;
   array(i,6) = leg_angle;
   array(i,7) = hip_torque;
   array(i,8) = leg_angle_desired;
   array(i,9) = xd;
 end;

% hack to keep it in view
 if ( x > max_x )
  x = x - (max_x - min_x); 
  foot_x = foot_x - (max_x - min_x); 
 end;

 if ( x < min_x )
  x = x + (max_x - min_x); 
  foot_x = foot_x + (max_x - min_x); 
 end;
 draw();

% have we crashed?
 if y < 0.1
   break;
 end;

% keep track of how long before crash.
 if ( i <= max_n_points )
   max_i = i; 
 end;
end

% plot data
figure; plot(array(1:max_i,1),array(1:max_i,2)) ;
    xlabel('Time (s)') ; ylabel('m') ; title('Vertical position (y)') ; grid on ;

figure ; plot(array(1:max_i,1),array(1:max_i,7)) ;
    xlabel('Time (s)') ; ylabel('Nm') ; title('Hip Torque') ; grid on ;

figure ; plot(array(1:max_i,1),array(1:max_i,6),'b', ...
              array(1:max_i,1),array(1:max_i,8),'r')
    xlabel('Time (s)') ; ylabel('m') ; title('Leg Angle') ; legend('actual', 'desired') ; grid on ;

figure ; plot(array(1:max_i,1),array(1:max_i,9)) ;
    xlabel('Time (s)') ; ylabel('m/s') ; title('Horizontal Speed') ; grid on ;
    
figure ; plot(array(1:max_i,1),array(1:max_i,4)) ;
    xlabel('Time (s)') ; ylabel('m/s') ; title('phase') ; grid on ;
    
figure ; 
plot(delta_xs ) 


%     xopt = fmincon(@opt_fun , [sign(L_desired)*abs(leg_angle_desired), height_desired, 0.1],...
%                             [],[],[],[], [-10*pi,0.5, -abs(L_desired)*0.05],...
%                             [10*pi,1.15, abs(L_desired)*0.05], @nonlcon,options) 
%     did_you_optimize_for_this_jump = not(did_you_optimize_for_this_jump) ; 
%     leg_angle_desired = xopt(1) ; 
%     height_desired = xopt(2) ;

% elseif not(jump_counter ==1 ) && not(did_you_optimize_for_this_jump) 
%     %speed_desired =-xopt(1) ; 
%     
%     leg_angle_desired =xopt(1) - pi  ; 


