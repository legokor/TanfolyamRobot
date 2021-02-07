log = table2array(importfile("log.csv"));
t1 = log(:,1);
setval = log(:,2);
encval = log(:,3);

speed = 0 * encval;

t = t1;
offset = 0;

for i=2:length(t1)
    if t1(i) - t1(i-1) < 0
       offset = offset + 65536;
    end

    t(i) = t1(i) + offset;

    speed(i) = encval(i) / (t(i)-t(i-1));
end

t = t - t(1);
t = t / 40000;

ti = (t(1):0.001:t(end))';
si = interp1(t, speed, ti);
vi = interp1(t, setval, ti);

t = ti;
speed = si;
setval = vi;

yyaxis left
plot(t, speed);
hold on;
yyaxis right
plot(t, setval);
