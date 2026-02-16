tic

t_prev = toc;

while true
    t = toc;
    while t - t_prev < 1.0
        t = toc;
    end
    t_prev = t;
    disp(t)
end
        