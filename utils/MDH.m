function [ G ] = MDH( a, alpha, d, theta )
    Ta = eye(4);
    Ta(1,4) = a;
    Ralpha = eye(4);
    Ralpha(1:3, 1:3) = ROTX(alpha);
    Td = eye(4);
    Td(3,4) = d;
    Rtheta = eye(4);
    Rtheta(1:3, 1:3) = ROTZ(theta);
    G = Ralpha * Ta * Rtheta * Td;
    % G = Ta * Ralpha * Td * Rtheta;
end