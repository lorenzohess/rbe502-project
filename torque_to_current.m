function I = torque_to_current(tau)
%TORQUE_TO_CURRENT_PIECEWISE Piecewise torque-current model
    k = 0.616;
    If = 0.00; % 100ma breakaway overcome static friction

    I = k .* tau + If .* sign(tau);
end
