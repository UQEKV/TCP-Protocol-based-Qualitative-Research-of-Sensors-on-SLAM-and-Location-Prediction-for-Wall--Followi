data = xlsread('accData.xlsx');

l1 = data(:, [8 9 7])';
% l1 = reshape(l1,[], 1);

l2 = data(:, [11 12 10])';
% l2 = reshape(l2,[], 1);

l3 = data(:, [14 15 13])';
% l3 = reshape(l3,[], 1);

l4 = data(:, [17 18 16])';
% l4 = reshape(l4,[], 1);

l5 = data(:, [2 3 1])';
% l5 = reshape(l5,[], 1);

l6 = data(:, [5 6 4])';
% l6 = reshape(l6,[], 1);

ObjFcn = @(m) ObjectFunction(m,data);
options = optimset('display','iter','TolX',1e-10,'MaxIter',100000,'MaxFunEvals',10000);
[m,fval] = fmincon(ObjFcn,rand(12,1),[],[],[],[],[],[],[],options );


M = inv([ m(1) m(2) m(3);
      m(4) m(5) m(6);
      m(7) m(8) m(9) ]);

b = [m(10); m(11); m(12)];

M * (l5 - b);
  

 function [Loss] = ObjectFunction(m, data)

g = 9.8118;

a1 = [g; 0; 0];
a2 = [-g; 0; 0];
a3 = [0; g; 0];
a4 = [0; -g; 0];
a5 = [0; 0; g];
a6 = [0; 0; -g];

M = [ m(1) m(2) m(3);
      m(4) m(5) m(6);
      m(7) m(8) m(9) ];           
M_all = repmat(M, size(data, 1), 1);

B = [m(10); m(11); m(12)];
B_all = repmat(B, size(data, 1), 1);

l1 = data(:, [8 9 7])';
l1 = reshape(l1,[], 1);

l2 = data(:, [11 12 10])';
l2 = reshape(l2,[], 1);

l3 = data(:, [14 15 13])';
l3 = reshape(l3,[], 1);

l4 = data(:, [17 18 16])';
l4 = reshape(l4,[], 1);

l5 = data(:, [2 3 1])';
l5 = reshape(l5,[], 1);

l6 = data(:, [5 6 4])';
l6 = reshape(l6,[], 1);

loss1 = l1 - B_all - M_all * a1 ;
loss2 = l2 - B_all - M_all * a2 ;
loss3 = l3 - B_all - M_all * a3 ;
loss4 = l4 - B_all - M_all * a4 ;
loss5 = l5 - B_all - M_all * a5 ;
loss6 = l6 - B_all - M_all * a6 ;

loss = [ loss1;
         loss2;
         loss3;
         loss4;
         loss5;
         loss6 ];
     
Loss = sum(sqrt(loss .* loss)) / (size(loss,1));


end





