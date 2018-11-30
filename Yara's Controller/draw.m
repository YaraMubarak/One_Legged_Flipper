function pos = draw()
% draw hopper

global h_axes body leg min_x max_x
global dt time x y xd yd 
global hip_torque leg_angle body_angle leg_angled body_angled
global leg_state foot_x foot_y leg_lengthd leg_length rest_leg_length
global Frame_array Frame_counter

body_length = 0.2;
bdx = 0.5*body_length*cos( body_angle );
bdy = 0.5*body_length*sin( body_angle );

set(body,'Parent',h_axes,'Xdata',[x - bdx x + bdx], ...
'Ydata',[y - bdy y + bdy],'visible','on');
set(leg,'Parent',h_axes,'Xdata',[x foot_x], ...
'Ydata',[y foot_y],'visible','on');
set(gca,'nextplot','replacechildren'); 

Frame_array{Frame_counter}= getframe; 
Frame_counter = Frame_counter + 1 ; 
drawnow

end
