function d = compress(d,q)

d = sign(d) .* (abs(d) .^ q);

