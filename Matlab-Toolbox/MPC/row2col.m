function vec = row2col(v)
%Transform a multidimensional row vector into a multidimensional column vector
[f,c] = size(v);
if f < c
    vec = v';
else
    vec = v;
end

