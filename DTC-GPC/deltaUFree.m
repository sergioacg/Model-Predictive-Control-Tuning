% By: Sergio Andres Castaño Giraldo
% http://controlautomaticoeducacion.com/
% ______________________________________________________________________
% Function to obtain the matrix of past controls for a MIMO system (for GPC)
% uG = deltaUFree(B, En, N, dp)
% uG  = Matrix of past controls
% B   = Numerator polynomial of the CARIMA model (CELL)
% En  = Diofantina E obtained from the diophantineMIMO function
% N   = Prediction Horizon
% dp  = Matrix with system delays
%
function uG = deltaUFree(B, En, N, dp)
    dmin = min(dp'); % Stores the minimum delay in a vector
    [ny, nu] = size(B);
    
    for m = 1:ny % Output counter
        for n = 1:nu % Input counter
            % Create a matrix to store the past control increments for each input-output relationship.
            % This matrix is later condensed into a single matrix.
            
            % The matrix of past controls depends only on the size of the numerator polynomial (B) and the delay of the system.
            % Thus, we can determine the dimensions of the matrix based on the prediction horizon for each output.
            % The number of rows is given by N(m), the prediction horizon for the m-th output.
            % The number of columns is determined by the size of B and the delay minus one.
            cp = (dp(m,n)) + length(B{m,n}) - 1; % Number of past data points
            
            if isempty(cp) || cp < 1
                cp = 1;
            end
            
            uG1 = zeros(N(m), cp); % (Nxcp) Matrix of past controls (belongs to the free response)
            
            % The data in the matrix of past controls is obtained by convolving the polynomials in En with B.
            for i = 1:N(m)
                aux = conv(En{m}(i,:), B{m,n});
                
                % The result stored in aux may contain trailing zeros.
                % We need to remove these trailing zeros from the vector.
                BE = [];
                
                for j = length(aux):-1:1
                   if aux(j) ~= 0
                       BE = [aux(j) BE];
                   end
                end
                
                % After removing the trailing zeros, we fill the uG1 matrix with the past control data from right to left.
                % The data is taken from the vector BE, but only the values related to the past.
                % These values correspond to the elements in BE from the end to the length "cp".
                lBE = length(BE); % Length of BE
                
                if lBE < cp
                    uG1(i,:) = [zeros(1, cp - lBE) BE];
                else 
                    uG1(i,:) = BE(end - cp + 1:end);
                end
                
            end
            
            uG{m,n} = uG1; % Store the matrix uG1 in a cell array
        end
    end
end
