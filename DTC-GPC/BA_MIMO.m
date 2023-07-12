% By: Sergio Andres Castaño Giraldo
% http://controlautomaticoeducacion.com/
% ______________________________________________________________________
% This function is responsible for obtaining the B matrix (MIMO numerator) and the A matrix
% (MIMO denominator) for the calculation of the diophantine equations in MPC. The input parameters
% are cell arrays Bn and An, which contain the numerators and denominators of the transfer function respectively.
% A quick way to obtain these cell arrays is by using the "descomp" function.
% [B,A,na,nb] = BA_MIMO(Bn,An)
% B    = B matrix
% A    = A matrix (Least Common Multiple)
% na   = Vector containing the sizes of the A polynomials
% nb   = Matrix containing the sizes of the B polynomials
% Bn   = Cell matrix containing the numerators of the MIMO transfer function
% An   = Cell matrix containing the denominators of the MIMO transfer function

function [B,A,na,nb] = BA_MIMO(Bn,An)
    [p,m] = size(An); % Number of outputs (p) and number of inputs (m)

    % Remove the leading zero from the numerator
    for i = 1:p
        for j = 1:m
            if Bn{i,j}(1) == 0 %&& length(Bn{i,j}) ~= 1
                Bn{i,j} = Bn{i,j}(2:end);
            end
        end
    end

    % Fill the diagonal of A with the least common multiple
    for i = 1:p
        aux = An{i,1};
        for j = 2:m
            aux = conv(aux, An{i,j});
        end
        A{i,i} = aux;

        % Check for multiple poles in the same row (output) and remove them from the least common multiple (MIMO case)
        if p ~= 1
            Au1 = round(roots(aux), 4);
            Pol = unique(Au1);
            A{i,i} = poly(Pol);
        end
    end

    % Fill the B cell array using the least common multiple (A)
    for i = 1:p
        for j = 1:m
            aux = Bn{i,j};
            rA = round(roots(A{i,i}), 4);
            rAn = round(roots(An{i,j}), 4);
            kk = 1;
            while (kk <= length(rA))
                for jj = 1:length(rAn)
                    if (rA(kk) == rAn(jj))
                        rA = rA(find(rA ~= rA(kk)));
                    end
                end
                kk = kk + 1;
            end
            pA = poly(rA);
            aux = conv(aux, pA);
            B{i,j} = aux;
        end
    end

    % Calculate the sizes of the A and B polynomials
    for i = 1:p
        na(i) = length(A{i,i}) - 1;
        for j = 1:m
            nb(i,j) = length(B{i,j}) - 1;
        end
    end
end
