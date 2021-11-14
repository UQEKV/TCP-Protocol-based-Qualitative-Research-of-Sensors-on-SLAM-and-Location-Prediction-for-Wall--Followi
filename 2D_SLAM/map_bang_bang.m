clear
close
load("1228.mat")

figure(1)
plot(M(:,1),M(:,2),"k-")
axis([-6000 1000 -2500 2000])
title("Map")
xlabel("X/mm")
ylabel("Y/mm")