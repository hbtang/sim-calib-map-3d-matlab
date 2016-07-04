syms x y
p = [x-1; y-2]

r = 0.5*p.'*[2 3;3 4]*p + [8 11]*p + 15;
simplify(r)