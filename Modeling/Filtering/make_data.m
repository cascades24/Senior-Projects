clear;close;clc;

n = 115;
noise = 6;
for i = 1:n
    x(i) =  10+rand*noise -noise/2;
    y(i) =  10+rand*noise -noise/2;
    z(i) =  10+rand*noise -noise/2;
end

mat = [x' y' z'];

dlmwrite('stationary.txt',mat,',')  


for i = 1:n
    x(i) =  10+rand*2 -1 + 0.1*i;
    y(i) =  10+rand*2 -1;
    z(i) =  10+rand*2 -1;
   
end

mat = [x' y' z'];

dlmwrite('motion.txt',mat,',')  

for i = 1:n
    x(i) =  10+rand*.1 +0.1*i;
    y(i) =  10+rand*.1 +0.1*i;
    z(i) =  10+rand*.1 +0.1*i;
end

mat = [x' y' z'];

dlmwrite('total_motion.txt',mat,',')  