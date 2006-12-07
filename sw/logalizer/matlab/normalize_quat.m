
function [quat_out] = normalize_quat(quat_in)

quat_out = quat_in / norm(quat_in);
