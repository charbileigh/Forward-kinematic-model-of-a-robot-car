pose = [0 0 0]';    %initial pose in frame {W}
dT = .01;           %sample time in seconds
T_end = 5;          %simulation duration
T = linspace(0,T_end,T_end/dT); %linearly spaced array of time points, from 0 to 5 seconds, in steps of 0.01.

p_des = [10 10]';   %desired x-y position in frame {W}. 
psi_ref = -45*pi/180; %desired final orientation for Q4. You can change this if you are testing other orientations.

cnt=0;
lwheel_vel = 10;
rwheel_vel = 10;
phi = pi/4;

wheel_radius = 30;
wheels_distance = 50;

k1 = 1.5;
k2 = 1.5;

trail = zeros(2, 10);
trail = trail';

figure(1),clf
for ind=1:length(T)%loop makes a timestep of dT, from 0 to 5 seconds
    
    t = T(ind); %current time
    
    %ADD CONTROLLER AND POSE UPDATE FUNCTIONS HERE
    [updated_pose, pose_derivative] = forward_kinematic(pose, lwheel_vel, rwheel_vel, phi, dT, wheel_radius, wheels_distance);
    
    [left_wheel_vel, right_wheel_vel, error_rot_vel] = position_controller(pose, p_des, wheel_radius, wheels_distance, k1, k2);
    lwheel_vel = left_wheel_vel;
    rwheel_vel = right_wheel_vel;
    phi = error_rot_vel;
    
    trail(:, 1) = pose(1);
    trail(:, 2) = pose(2);
    
    cnt = cnt + 1;
    
    if(cnt==10) %the simulation runs at 100Hz (dT=0.01), but we will plot at 10Hz
        cnt = 0;
        plot(trail(:,1), trail(:,2), ".b");
        pose = updated_pose;
        clf
        plotTrVec = [pose(1:2); 0]; %3x1 translation vector that is made up of the current position: pose(1:2)
        plotRot = [cos(pose(3)/2) 0 0 sin(pose(3)/2)];  %construct quaternion from orientation: pose(3)
        
        xlim([0 20]),ylim([-2 20]) %fixes the x-y axes ranges
        %plots the body-frame coordinate system and also adds a visualisation of our mobile robot
        plotTransforms(plotTrVec', plotRot, "MeshFilePath", "groundvehicle.stl", "Parent", gca, "View","2D", "FrameSize", 1);
        
        %Visual indication that the controller meets the specs in Q2 (car will turn red). Comment this out when you move to Q4.
        if( norm(pose(1:2)-p_des)>1e-2 ) 
            light;  %car icon is set to white until tolerances are met. 
        end
         %Visual indication that the controller meets the specs in Q4 (car will turn red). Uncomment this when you move to Q4.
%         if( (norm(pose(1:2)-p_des)>1e-2) || (norm(psi_ref-pose(3))>5*pi/180 ) )
%             light;    %car icon is set to white until tolerances are met.
%         end
        hold on
        plot(p_des(1,end),p_des(2,end),'or','markerSize',25)    %plots our target destination as a circle.
        
        %Uncomment the line below when performing Q4. It will display the desired pose of frame {B} wrt frame {W}.
%         plotTransforms([p_des(:,end)' 0], [cos(psi_ref/2) 0 0 sin(psi_ref/2)], "Parent", gca, "View","2D", "FrameSize", 1)
        drawnow     %forces MATLAB to render the figures immediately.
        
    end   

end

function [updated_pose, pose_derivative] = forward_kinematic(pose, lwheel_vel, rwheel_vel, phi, dT, wheel_radius, wheels_distance)
    vel_x = (lwheel_vel + rwheel_vel)*cos(phi)*wheel_radius*0.5;
    vel_y = (lwheel_vel + rwheel_vel)*sin(phi)*wheel_radius*0.5;
    angular_vel = (lwheel_vel - rwheel_vel)*(wheel_radius/wheels_distance);
    
    linear_vel = [vel_x; vel_y; angular_vel];
    pose_derivative = [vel_x; vel_y; angular_vel];
    rotation_matrix = [cos(angular_vel) sin(angular_vel) 0; sin(angular_vel) cos(angular_vel) 0; 0 0 1]*linear_vel;
    updated_pose = pose + rotation_matrix*dT;
end

function [left_wheel_vel, right_wheel_vel, error_rot_vel] = position_controller(pose, p_des, wheel_radius, wheels_distance, k1, k2)
    error_x = abs(p_des(1) - pose(1));
    error_y = abs(p_des(2) - pose(2));
    
    wheel_vel = k1*sqrt(error_x*error_x + error_y*error_y);
    error_rot_vel = k2*(atan2(error_y,error_x) - pose(3));
    vel = [wheel_vel; error_rot_vel];
    matrix_radii = [wheel_radius/2 wheel_radius/2; wheel_radius/wheels_distance -wheel_radius/wheels_distance];
                
    left_right_vel = inv(matrix_radii)*vel;
    left_wheel_vel = abs(left_right_vel(1));
    right_wheel_vel = abs(left_right_vel(2));
end

function  [x, y] = trajectory_generation(pose, p_des)
    initial_final_pose = [pose(1) p_des(1); pose(2) p_des(2)];
    [x, a1, a2, y, a3] = trapveltraj(initial_final_pose, 500, 'Endtime', 5);
end