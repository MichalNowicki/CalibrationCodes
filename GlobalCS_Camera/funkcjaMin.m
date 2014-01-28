function F = funkcjaMin( X )
% Global equations
global Rhs;
global Lhs;
   % Recalculation to rotation matrix
   W = rodrigues(X);
   % Equation error
   F = abs(Rhs - W * Lhs);
end

