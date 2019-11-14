clear;close;clc;

n = 1415;
   
for i = 1:n
    x(i) =  10+rand*.1;
    y(i) =  10+rand*.1;
    z(i) =  10+rand*.1;
end

mat = [x' y' z'];

dlmwrite('myData.txt',mat,',')  