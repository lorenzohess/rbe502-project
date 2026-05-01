function [qddot, qdot_filt] = qddot_filter(qdot, qdot_filt, qdot_prev_filt, alpha, dt)
    qdot_filt = alpha*qdot + (1-alpha)*qdot_prev_filt;
    qddot = (qdot_filt - qdot_prev_filt) / dt;
end
