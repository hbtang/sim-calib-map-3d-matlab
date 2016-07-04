function spy_sym( A )
%SPY_SYM spy of a symbolic matrix
figure;

sz = size(A);
B = ones(sz);

for i = 1:sz(1)
    for j = 1:sz(2)
        if A(i,j) == 0
            B(i,j) = 0;
        end
    end
end

spy(B);

end

