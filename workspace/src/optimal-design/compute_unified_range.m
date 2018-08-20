function output = compute_unified_range(r,d,theta)
a = 1;
b = -2*d*cos(theta);
c = d^2-r^2;
d= b^2 - 4 * a * c;
if   d> 0
    x1 = (-b + sqrt(d)) / (2*a);
    x2 = (-b - sqrt(d)) / (2*a);
end

output = max(x1,x2);