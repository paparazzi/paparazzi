%
% plot a serie of measures realised with the black 10*4.5 prop
%

function [] = plot_prop()

    rpm =      [ 2800 3350 3720 4450 5250 ];
    thrust_g = [  122  175  219  310  445 ];
    torque_g = [   10   16   19   26   44 ];

    omega = rpm / 60 * 2 * pi;
    omega_square = omega.^2;

    thrust_n = thrust_g .* ( 9.81 / 1000 ); 
    torque_n = torque_g .* ( 9.81 / 1000 ); 

    K = fminsearch(@lin_model, [1, 1]);
    Kt = K(1);
    Kq = K(2);
    
    % F = 0.5 * rho * prop_area * ct * prop_rad^2 * omega^2
    rho = 1.225;
    prop_area = 0.005;
    prop_rad = 0.125;
    Ct = Kt / (0.5 * rho * prop_area * prop_rad^2)  
    Cq = Kq / (0.5 * rho * prop_area * prop_rad^2)
    
    plot(omega_square, thrust_n, ...
         omega_square, Kt * omega_square, ...
         omega_square, torque_n, ...
         omega_square, Kq * omega_square )
     
    title('Propeller caracterisation (EPP 10*4.5)');
    legend('thrust', 'fitted thrust', 'torque', 'fitted torque');
    xlabel('omega square in rad^2/s^2');
    ylabel('forces in N');
    tb = annotation('textbox', [0.2 0.65 0.1 0.1]);
    set(tb, 'String', ...
        [sprintf('Thrust = %.2e * omega^2 ( Ct = %.2e )\n\n', Kt, Ct) ...
         sprintf('Torque = %.2e * omega^2 ( Cq = %.2e )', Kq, Cq)], ... 
        'FitHeightToText', 'on' );
    
    function [sse] = lin_model(params)
        Kt = params(1);
        Kq = params(2);
        fitted_thrust = Kt * omega_square;        
        fitted_torque = Kq * omega_square;
        err_t = fitted_thrust - thrust_n;
        err_q = fitted_torque - torque_n;
        sse = sum(err_t .^ 2) + sum(err_q .^ 2);
    end

end