function [jump_end] = check_jump(control_state,prev_state) 
jump_end = false ;
if not(control_state == prev_state) 
control_state;
prev_state;
end 

if control_state == prev_state 
    jump_end = false ;
elseif control_state == 2
    jump_end = true ; 
end

if control_state == 2 
    disp('')  
end 

end 
