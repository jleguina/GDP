%% Housekeeping
clear
clc
close all

% Note - coordinate axes
%       y:          North
%       x:          East
%       z:          up
%  angles:          positive clockwise between -pi & pi

%% Constants - angles in degrees
g = 9.81;
Rd = 30;

% Blue parameters
phi_dot_b = 15;
phi_max_b = 45;
theta_dot_b = 15;
theta_max_b = 45;
n_b = 1;
acc_max_b = 2;
dec_max_b = -1;

% Red parameters
phi_dot_r = 15;
phi_max_r = 45;
theta_dot_r = 15;
theta_max_r = 45;
n_r = 1;
acc_max_r = 2;
dec_max_r = -1;

% Initial position
xb = 100;
yb = 10;
zb = 100;
phi_b = 0;
theta_b = 0;
psi_b = 0;
Vb = 10;

xr = 100;
yr = 0;
zr = 100;
phi_r = 0;
theta_r = 0;
psi_r = 0;
Vr = 10;

%% Simulation loop
Timecount = 1;
for time = 1:500
    
    %% Control inputs:
    %   u_phi   = {-1, 0, +1} - Roll left, no roll, roll-right
    %   u_theta = {-1, 0, +1} - Pitch-down, no pitch, pitch-down
    %   u_vel   = {-1, 0, +1} - Deccelerate, no-acceleration, accelerate
    
    % Blue
    u_phi_b = [-1, 0, +1];
    u_theta_b = [-1, 0, +1];
    u_vel_b = [-1, 0, +1];
    
    % Red
    u_phi_r = [-1, 0, +1];
    u_theta_r = [-1, 0, +1];
    u_vel_r = [-1, 0, +1];
    
    %% Equations of Motion and Score Matrix
    % t_0: start of integration
    % t_f: end of integration
    %  dt: timestep
    t_0 = 0;
    dt = 0.25;
    t_f = 1.5;
    
    % Set counter for blue maneuvres
    countB = 1;
    
    for i = 1:3 % Blue phi (roll)
        for j = 1:3 % Blue theta (pitch)
            for ij = 1:3 % Velocity blue
                % Set counter for red maneuvres
                countR = 1;
                
                for k = 1:3 % Red phi (roll)
                    for m = 1:3 % Red theta (pitch)
                        for km = 1:3 % Velocity red
                            
                            % Velocity adavancements for blue and red
                            if u_vel_r(km) < 0
                                a_r = abs(dec_max_r);
                            elseif u_vel_r(km) == 0
                                a_r = 0;
                            elseif u_vel_r(km) > 0
                                a_r = acc_max_r;
                            end
                            
                            if u_vel_b(ij) < 0
                                a_b = abs(dec_max_b);
                            elseif u_vel_b(ij) == 0
                                a_b = 0;
                            elseif u_vel_b(ij) > 0
                                a_b = acc_max_b;
                            end
                            
                            Vr_new = Vr + u_vel_r(km)*a_r*dt;
                            Vb_new = Vb + u_vel_b(ij)*a_b*dt;
                            
                            %% Time Integration
                            for t=t_0:dt:t_f %Assume V_x and control input is constant during look ahead time
                                %%%%% BLUE AIRCRAFT
                                % ROLL
                                % Advancement of phi; phi_dot - maximum roll rate
                                phi_b_test = phi_b + u_phi_b(i)*phi_dot_b*dt;
                                
                                % Phi stays within acceptable values
                                phi_b_test = max(phi_b_test,-phi_max_b);
                                phi_b_test = min(phi_b_test, phi_max_b);
                                
                                if abs(phi_b_test)>180
                                    phi_b_test = wrapTo180(phi_b_test); % Wrap angle
                                end
                                
                                % PITCH
                                % Advancement of theta; theta_dot - maximum pitch rate
                                theta_b_test = theta_b + u_theta_b(j)*theta_dot_b*dt;
                                
                                % Theta stays within acceptable values
                                theta_b_test = max(theta_b_test,-theta_max_b);
                                theta_b_test = min(theta_b_test, theta_max_b);
                                
                                if abs(theta_b_test)>180
                                    theta_b_test = wrapTo180(theta_b_test); % Wrap angle
                                end
                                
                                % HEADING
                                psi_dot_b_test = 180/pi*g*n_b*tand(phi_b)/Vb_new;
                                psi_b_test = psi_b + psi_dot_b_test*dt;
                                
                                if abs(psi_b_test)>180
                                    psi_b_test = wrapTo180(psi_b_test); % Wrap angle
                                end
                                
                                %%%%% RED AIRCRAFT
                                % ROLL
                                % Advancement of phi; phi_dot - maximum roll rate
                                phi_r_test = phi_r + u_phi_r(k)*phi_dot_r*dt;
                                
                                % Phi stays within acceptable values
                                phi_r_test = max(phi_r_test,-phi_max_r);
                                phi_r_test = min(phi_r_test, phi_max_r);
                                
                                if abs(phi_r_test)>180
                                    phi_r_test = wrapTo180(phi_r_test); % Wrap angle
                                end
                                
                                % PITCH
                                % Advancement of theta; theta_dot - maximum pitch rate
                                theta_r_test = theta_r + u_theta_r(m)*theta_dot_r*dt;
                                
                                % Theta stays within acceptable values
                                theta_r_test = max(theta_r_test,-theta_max_r);
                                theta_r_test = min(theta_r_test, theta_max_r);
                                
                                if abs(theta_r_test)>180
                                    theta_r_test = wrapTo180(theta_r_test); % Wrap angle
                                end
                                
                                % HEADING
                                psi_dot_r_test = 180/pi*g*n_r*tand(phi_r_test)/Vr_new;
                                psi_r_test = psi_r + psi_dot_r_test*dt;
                                
                                if abs(psi_r_test)>180
                                    psi_r_test = wrapTo180(psi_r_test); % Wrap angle
                                end
                            end
                            
                            % New coordinates are:
                            xb_test = xb + Vb_new*dt*cosd(theta_b_test)*sind(psi_b_test);
                            yb_test = yb + Vb_new*dt*cosd(theta_b_test)*cosd(psi_b_test);
                            zb_test = zb + Vb_new*dt*sind(theta_b_test);
                            
                            xr_test = xr + Vr_new*dt*cosd(theta_r_test)*sind(psi_r_test);
                            yr_test = yr + Vr_new*dt*cosd(theta_r_test)*cosd(psi_r_test);
                            zr_test = zr + Vr_new*dt*sind(theta_r_test);
                            
                            % Range
                            R = sqrt((xb_test-xr_test)^2+(yb_test-yr_test)^2+(zb_test-zr_test)^2);
                            
                            % Angles:
                            % Angle between Blue-Red line and y-axis:
                            if yr_test < yb_test
                            alpha = abs(atand(abs(xr_test-xb_test)/abs(yr_test-yb_test)));
                            elseif yb_test < yr_test
                            alpha = abs(atand(abs(xb_test-xr_test)/abs(yb_test-yr_test)));  
                            end
                                
                            % Angle between Blue-Red line and x-z plane:
                            vector = [abs(xr_test-xb_test), abs(yr_test-yb_test), abs(zr_test-zb_test)];
                            norm_vector = norm(vector);
%                             y_axis = [0,1,0];
%                             norm_y = norm(y_axis);
%                             alpha = abs(acosd(sum(vector.*y_axis)/(norm_vector*norm_y)));
                            
                            % Angle between Blue-Red line and x-y plane:
                            z_axis = [0,0,1];
                            norm_z = norm(z_axis);
                            beta = abs(acosd(sum(vector.*z_axis)/(norm_vector*norm_z)));
                            
                            % Hence:
                            ATAh = abs(alpha + psi_b_test);
                            if ATAh>180
                                ATAh = abs(wrapTo180(ATAh));
                            end
                            
                            AAh =  abs(alpha + psi_r_test);
                            if AAh>180
                                AAh = abs(wrapTo180(AAh));
                            end
                            
                            ATAv = abs((90 - beta) - phi_b_test);
                            if ATAv>180
                                ATAv = abs(wrapTo180(ATAv));
                            end
                            
                            AAv = abs((90 - beta) - phi_r_test);
                            if AAv>180
                                AAv = abs(wrapTo180(AAv));
                            end
                            %% Score Function - Orientation
                            % Horizontally:
                            %    AAh:    aspect angle
                            %   ATAh:    antenna train angle
                            
                            %  For red:     AAh = ATAh = 0     -->    Sa = 1
                            %  For blue:    AAh = ATAh = 180  -->    Sa = -1
                            Sah = 1 - abs(AAh+ATAh)/180;
                            
                            % Vertically:
                            %    AAv:    aspect angle
                            %   ATAv:    antenna train angle
                            
                            %  For red:     AA = ATA = 0     -->    Sa = 1
                            %  For blue:    AA = ATA = 180  -->    Sa = -1
                            Sav = 1-abs(AAv+ATAv)/180;
                            
                            % Assembled:
                            Sa = 0.5*(Sav+Sah);
                            
                            
                            %% Score Function - Range
                            %     R:    distance between aircraft
                            %    Rd:    desired distance between aircraft
                            %    er:    distance error
                            %     k:    constant
                            
                            er = abs(R-Rd);
                            
                            % Range score for each aircraft
                            Sr = exp(-er/(180));
                            
                            %% Score Function - Velocity
                            %           bd:     desired velocity
                            %   b0, b1, bc:     curve parameters
                            %           Vx:     velocity of aircraft
                            %     CVb, CVr:     weights for scores
                            %           Rc:     combat range
                            %           Rd:     desired distance between aircraft
                            %            a:     acceleration value
                            
                            % Combat range is such that constant maximum de/acceleration can be applied for a time
                            % t and the resulting speed is Vb exactly at Rd.
                            %Rc = (Vb_new - Vr_new)/a_r + Rd;
                            Rc = 0.5;
                            % Value of a is max. acceleration (a_r > 0) if Vr < Vb
                            % Value of a is max. deceleration (a_r < 0 ) if Vr > Vb
                            if Vr_new < Vb_new
                                a_r = acc_max_r;
                            elseif Vr_new > Vb_new
                                a_r = dec_max_r;
                            end
                            
                            % Estimation of desired V as a function of R for
                            % red aircraft
                            if R <= Rd
                                bd = Vb_new;
                            elseif R < Rc & R > Rd
                                bd = a_r*er + Vr_new;
                            elseif R > Rc
                                bd = 1; %Guess- bd = max. endurance?
                            end
                            
                            b0 = 0.1*bd;
                            b1 = 0.3*bd;
                            bc = 3*bd;
                            
                            % Absolute contribution
                            Sv_b = (1-((b1-b0)/(b1+Vb_new))^2)*exp(-(Vb_new-bd)^2/bc^2);
                            Sv_r = (1-((b1-b0)/(b1+Vr_new))^2)*exp(-(Vr_new-bd)^2/bc^2);
                            
                            % Function weights
                            CVb = 1;
                            CVr = 1;
                            
                            %   For red:    Sv = 1
                            %   For blue:   Sv = -1
                            Sv = CVr*Sv_r - CVb*Sv_b;
                            
                            %% Score Function - Terrain
                            %           ad:     desired clearance
                            %   a0, a1, ac:     curve parameters
                            %            z:     altitude of aircraft
                            %     CHb, CHr:     weights for scores
                            
                            ad = 2;   %Guess - must revise
                            a1 = 0.5;    %Guess - must revise
                            a0 = (2*a1+ad)/300;
                            ac = ad-a1;
                            
                            % To avoid penalisisng high altitudes, evaluate function at ad (S = 1) if z > ad:
                            % For terrain following, penalise when z > ad.
                            if zr >= ad
                                Sh_r = 1;
                            elseif zr < ad
                                Sh_r = (1-((a0-b1)/(zr-a1))^2)*exp(-(zr-ad)^2/ac^2);
                            end
                            
                            if zb >= ad
                                Sh_b = 1;
                            elseif zb < ad;
                                Sh_b = (1-((a0-b1)/(zb-a1))^2)*exp(-(zb-ad)^2/ac^2);
                            end
                            % Function weights
                            CHb = 1;
                            CHr = 1;
                            
                            %   For red:    Sv = 1
                            %   For blue:   Sv = -1
                            Sh = CHr*Sh_r - CHb*Sh_b;
                            
                            
                            %% Scoring Function - Assembled
                            %S(countB,countR) = Sr*(Sa+Sh); %+ Sv;
                            S(countB,countR) = Sah;
                            
                            % Advance counter for red maneuvres
                            countR = countR + 1;
                        end
                    end
                end
                % Advance counter for blue maneuvres
                countB = countB + 1;
            end
        end
    end
    
    
    %% Max-min search - red wants to maximize, blue wants to minimize S
    [rowS, colS] = size(S);
    for i = 1:colS
        mins(i) = min(S(:,i));
    end
    
    maxs = max(mins);
    [~, col] = find(S==maxs);
    col = unique(col);
    
    if length(col) ~= 1
        col = col(ceil(rand*length(col)));
        %col = round(median(col));
    end
    
    %% Min-max search - blue wants to minimize
    [rowS, colS] = size(S);
    for i = 1:rowS
        maxs(i) = max(S(:,i));
    end
    
    mins = min(maxs);
    [row, ~] = find(S==mins);
    row = unique(row);
    
    if length(row) ~= 1
        row = row(ceil(rand*length(row)));
        %row = row(1);
    end
    
    S_count(Timecount) = S(row, col);
    
    % Locate control inputs that lead to the obtained row & column
    % RED
    % Velocity
    if mod(col,3)/3==0
        u_vel_r_fin = 1;
    elseif mod(col,3)/3== 2/3
        u_vel_r_fin = 0;
    elseif mod(col,3)/3== 1/3
        u_vel_r_fin = -1;
    end
    
    % Phi
    if ceil(col/9) == 3
        u_phi_r_fin = 1;
    elseif ceil(col/9) == 2
        u_phi_r_fin = 0;
    elseif ceil(col/9) == 1
        u_phi_r_fin = -1;
    end
    
    % Theta
    if ceil(ceil(col/3)/3) == 3
        u_theta_r_fin = 1;
    elseif ceil(ceil(col/3)/3) == 2
        u_theta_r_fin = 0;
    elseif ceil(ceil(col/3)/3) == 1
        u_theta_r_fin = -1;
    end
    
    % BLUE
    % Velocity
    if mod(row,3)/3==0
        u_vel_b_fin = 1;
    elseif mod(row,3)/3== 2/3
        u_vel_b_fin = 0;
    elseif mod(row,3)/3== 1/3
        u_vel_b_fin = -1;
    end
    
    % Phi
    if ceil(row/9) == 3
        u_phi_b_fin = 1;
    elseif ceil(row/9) == 2
        u_phi_b_fin = 0;
    elseif ceil(row/9) == 1
        u_phi_b_fin = -1;
    end
    
    % Theta
    if ceil(ceil(row/3)/3) == 3
        u_theta_b_fin = 1;
    elseif ceil(ceil(row/3)/3) == 2
        u_theta_b_fin = 0;
    elseif ceil(ceil(row/3)/3) == 1
        u_theta_b_fin = -1;
    end
    
    %     u_vel_b_fin = 0;
    %     u_phi_b_fin = 0;
    %     u_theta_b_fin = 0;
    
    %% Recalculate new positions
    % Velocity adavancements for red
    if u_vel_r_fin < 0
        a_r = abs(dec_max_r);
    elseif u_vel_r_fin == 0
        a_r = 0;
    elseif u_vel_r_fin > 0
        a_r = acc_max_r;
    end
    
    Vr_new_fin = Vr + u_vel_r_fin*a_r*dt;
    
    % Velocity adavancements for blue
    if u_vel_b_fin < 0
        a_b = abs(dec_max_b);
    elseif u_vel_b_fin == 0
        a_b = 0;
    elseif u_vel_b_fin > 0
        a_b = acc_max_b;
    end
    
    Vb_new_fin = Vb + u_vel_b_fin*a_b*dt;
    
    %% Time Integration
    for t=t_0:dt:t_f %Assume V_x and control input is constant during look ahead time
        %%%%% BLUE AIRCRAFT
        % ROLL
        % Advancement of phi; phi_dot - maximum roll rate
        phi_b = phi_b + u_phi_b_fin*phi_dot_b*dt;
        
        % Phi stays within acceptable values
        phi_b = max(phi_b,-phi_max_b);
        phi_b = min(phi_b, phi_max_b);
        if abs(phi_r)>180
            phi_b = wrapTo180(phi_b); % Wrap angle
        end
        
        % PITCH
        % Advancement of theta; theta_dot - maximum pitch rate
        theta_b = theta_b + u_theta_b_fin*theta_dot_b*dt;
        
        % Theta stays within acceptable values
        theta_b = max(theta_b,-theta_max_b);
        theta_b = min(theta_b, theta_max_b);
        if abs(theta_b)>180
            theta_b = wrapTo180(theta_b); % Wrap angle
        end
        
        % HEADING
        psi_dot_b = 180/pi*g*n_b*tand(phi_b)/Vb_new_fin;
        psi_b = psi_b + psi_dot_b*dt;
        if abs(psi_b)>180
            psi_b = wrapTo180(psi_b); % Wrap angle
        end
        
        %%%%% RED AIRCRAFT
        % ROLL
        % Advancement of phi; phi_dot - maximum roll rate
        phi_r = phi_r + u_phi_r_fin*phi_dot_r*dt;
        
        % Phi stays within acceptable values
        phi_r = max(phi_r,-phi_max_r);
        phi_r = min(phi_r, phi_max_r);
        if abs(phi_r)>180
            phi_r = wrapTo180(phi_r); % Wrap angle
        end
        
        % PITCH
        % Advancement of theta; theta_dot - maximum pitch rate
        theta_r = theta_r + u_theta_r_fin*theta_dot_r*dt;
        
        % Theta stays within acceptable values
        theta_r = max(theta_r,-theta_max_r);
        theta_r = min(theta_r, theta_max_r);
        if abs(theta_r)>180
            theta_r = wrapTo180(theta_r); % Wrap angle
        end
        
        % HEADING
        psi_dot_r = 180/pi*g*n_b*tand(phi_r)/Vr_new_fin;
        psi_r = psi_r + psi_dot_r*dt;
        if abs(psi_r)>0
            psi_r = wrapTo180(psi_r); % Wrap angle
        end
    end
    
    % New coordinates are:
    xr_fin = xr + Vr_new_fin*dt*cosd(theta_r)*sind(psi_r);
    yr_fin = yr + Vr_new_fin*dt*cosd(theta_r)*cosd(psi_r);
    zr_fin = zr + Vr_new_fin*dt*sind(theta_r);
    
    xb_fin = xb + Vb_new_fin*dt*cosd(theta_b)*sind(psi_b);
    yb_fin = yb + Vb_new_fin*dt*cosd(theta_b)*cosd(psi_b);
    zb_fin = zb + Vb_new_fin*dt*sind(theta_b);
    
    % Re-position aircraft to restart loop
    xr_pos(Timecount) = xr_fin;
    yr_pos(Timecount) = yr_fin;
    zr_pos(Timecount) = zr_fin;
    
    xr = xr_fin;
    yr = yr_fin;
    zr = zr_fin;
    
    % Re-position aircraft to restart loop
    xb_pos(Timecount) = xb_fin;
    yb_pos(Timecount) = yb_fin;
    zb_pos(Timecount) = zb_fin;
    
    xb = xb_fin;
    yb = yb_fin;
    zb = zb_fin;


subplot(2,1,1);
plot3(xr_pos,yr_pos,zr_pos,'r')
hold on
plot3(xb_pos,yb_pos,zb_pos,'b')
grid on
axis square


subplot(2,1,2);
plot(Timecount, S_count(Timecount),"-xk")
hold on
grid on
axis square

pause(0.5)
 Timecount = Timecount + 1;
end


%% Plot postion of aircraft
plot3(xr_pos,yr_pos,zr_pos,'r')
hold on
plot3(xb_pos,yb_pos,zb_pos,'b')
grid on
pause(0.5)

figure
subplot(2,1,1);
plot(xr_pos,yr_pos,'r');
hold on
plot(xb_pos,yb_pos,'b');

subplot(2,1,2);
plot(xr_pos,zr_pos,'r');
hold on
plot(xb_pos,zb_pos,'b');