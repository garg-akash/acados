clear all 
close all
M = readmatrix('eight_0.1.txt');
%M = readmatrix('fulleight50_iter.txt');
%plot
figure(1);
subplot(2,2,1)
plot(M(:,1));
title('x(1) X Position')
grid on
subplot(2,2,2)
plot(M(:,2));
title('x(2) Y Position')
grid on
subplot(2,2,3)
plot(M(:,3));
title('x(3) Orientation')
grid on
subplot(2,2,4)
plot(M(:,4));
title('x(4) Slider Positon (angle)')
grid on
%plot
figure(2);
subplot(2,2,1)
plot(M(:,5));
title('x(1) desired X Position')
grid on
subplot(2,2,2)
plot(M(:,6));
title('x(2) desired Y Position')
grid on
subplot(2,2,3)
plot(M(:,7));
title('x(3) desired Orientation')
grid on
subplot(2,2,4)
plot(M(:,8));
title('x(4) desired Pusher Positon (angle)')
grid on
%plot
figure(3);
subplot(2,2,1)
plot(M(:,1)-M(:,5));
title('x(1) X Position error')
grid on
subplot(2,2,2)
plot(M(:,2)-M(:,6));
title('x(2) Y Position error')
grid on
subplot(2,2,3)
plot(M(:,3)-M(:,7));
title('x(3) Orientation error')
grid on
subplot(2,2,4)
plot(M(:,4)-M(:,8));
title('x(4) Pusher Positon (angle) error')
grid on
%plot
figure(4);
subplot(2,2,1)
plot(M(:,9));
title('u(1) Normal force')
grid on
subplot(2,2,2)
plot(M(:,10));
title('u(2) Tangential force')
grid on
subplot(2,2,3)
plot(M(:,11));
title('u(3) Phi dot plus')
grid on
subplot(2,2,4)
plot(M(:,12));
title('u(3) phi dot minus')
grid on
figure(5);
plot(M(:,13));
title('computation time millisecond')
grid on
figure(6);
plot(M(:,1),M(:,2));
hold on
plot(M(:,5),M(:,6));
grid on
figure(7);
plot(M(:,14));
title('return status')
grid on
figure(8);
plot(M(:,15));
title('cost function')
grid on
