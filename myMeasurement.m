function y = myMeasurement(x)
% x = [phi;theta;psi;p;q;r]
ptp = [x(1);x(2);x(3)];
q0123 = Eu2Quat(ptp);
pqr = x(4:7);
y = [q0123;pqr]; 
end