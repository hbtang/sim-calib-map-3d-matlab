x = linspace(0,10);
y1 = sin(x);
y2 = sin(2*x);
y3 = sin(4*x);
y4 = sin(8*x);

figure
subplot(4,4,1)
plot(x,y1)
% title('Subplot 1: sin(x)')

subplot(4,4,6)
plot(x,y2)
% title('Subplot 2: sin(2x)')

subplot(4,4,11)
plot(x,y3)
% title('Subplot 3: sin(4x)')

subplot(4,4,16)
plot(x,y4)
% title('Subplot 4: sin(8x)')