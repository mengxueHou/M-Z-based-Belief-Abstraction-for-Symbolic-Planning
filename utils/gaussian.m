function f = gaussian(w, mu, sigma, x, y)
f = zeros(length(x), length(y));
dim = 2;
    for ii = 1:length(x)
        for jj = 1:length(y)
            f(ii,jj) = f(ii,jj) + w * 1/sqrt(((2*pi)^dim)*norm(sigma))*exp(-0.5*([x(ii); y(jj)]-mu)'*inv(sigma)*([x(ii); y(jj)]-mu));
        end
    end
end