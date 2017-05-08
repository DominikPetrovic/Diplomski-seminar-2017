function gama=map_gama(n,dt,w)

syms k

f=power(dt,n+k)/factorial(n+k)*power(skew_matrix(w),k);
gama = double(symsum(f,k,0,inf));

end