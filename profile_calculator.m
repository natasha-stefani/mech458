n = [3 2 7 2 3];

accv = [.000003 .00001 .000032 .00001 .000003]; 

curve = zeros(1,n(1) + n(2) + n(3) + n(4) + n(5));

curr_time = 14000.0;
curve(1) = curr_time;

curr_idx = 2;

for i = 1:size(n')
    for j = 1:n(i)
        curr_time = 1.8/(accv(i)+(1.8/curr_time));
        curve(curr_idx) = fix(curr_time);
        curr_idx = curr_idx + 1;
    end
end

curr_time = 14000.0;
rev_curve(1) = curr_time;
n(3) = n(3)/2;
a(3) = a(3)*2;
rev_curve = zeros(1,n(1) + n(2) + n(3) + n(4) + n(5));
for i = 1:size(n')
    for j = 1:n(i)
        curr_time = 1.8/(accv(i)+(1.8/curr_time));
        rev_curve(curr_idx) = fix(curr_time);
        curr_idx = curr_idx + 1;
    end
end

curve'
rev_curve'
vel = 1.8 ./ curve
plot(vel)