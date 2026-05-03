function R_gain = construct_R()
%CONSTRUCT_R  Build diagonal adaptation gain matrix for pi_hat.
%
%   R_gain = construct_R() returns a 16x16 diagonal R matrix.
%   Edit the per-parameter values directly below.
%
%   Larger R = slower learning. Smaller R = faster learning.
%   1e12 effectively freezes a parameter at its initial value.
%
%   Order matches pi_hat = p_bar(7:22).

    R_default = 1e12;

    %% Per-parameter adaptation gains
    Rm1 = R_default;
    Rm2 = R_default;
    Rm3 = R_default;
    Rm4 = R_default;

    RIxx1 = R_default;
    RIyy1 = R_default;
    RIzz1 = R_default;

    RIxx2 = R_default;
    RIyy2 = R_default;   % frozen — went unstable
    RIzz2 = R_default;

    RIxx3 = R_default;
    RIyy3 = R_default;
    RIzz3 = R_default;

    RIxx4 = R_default;
    RIyy4 = R_default;   % frozen — drifted to 16x
    RIzz4 = R_default;

    %% Build diagonal gain matrix
    R_gain = diag([Rm1; Rm2; Rm3; Rm4; ...
                   RIxx1; RIyy1; RIzz1; ...
                   RIxx2; RIyy2; RIzz2; ...
                   RIxx3; RIyy3; RIzz3; ...
                   RIxx4; RIyy4; RIzz4]);
end
