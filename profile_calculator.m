n = [3 2 6 2 7];

accv = [.000003 .00001 .000028 .00001 .000003]; 

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

curve'
vel = 1.8 ./ curve
plot(vel)