function d = clipdata(d,threshold)

index = find(d > threshold);
d(index) = threshold;

index = find(d < -threshold);
d(index) = -threshold;