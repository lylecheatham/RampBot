sample_freq = 20e6;
window      = 20;
order       = 3;
dt = 1.0/sample_freq;

% Set up t and t' vectors
t       = ones(1,order+1).*(window+1)*dt;
t_prime = ones(1,order+1).*(window+1)*dt;

for i = 1:(order+1)
  t(i) = t(i)^(order+1-i);
  t_prime(i) = (order+1-i)*t_prime(i)^((order-i));
endfor

% Set up T*
T = zeros(window, order+1);
ct = dt*window;
for i = 1:window
  for j = 1:order+1
    T(i,j) = ct^(order+1-j);  
  endfor
  ct -= dt;
endfor

T_star = inverse(T' * T) * T';


% Output in a format to paste into c code
printf("\n T_star = {\n");
for i = 1:order+1
  printf("{ ");
  for j = 1:window
    printf(" %f, ", T_star(i,j));
  endfor
  printf(" }, \n");
endfor
printf("};\n");

printf("\n t = {");
for i = 1:order+1
  printf(" %f, ", t(i));
endfor
printf("};\n");

printf("\n t_prime = {");
for i = 1:order+1
  printf(" %f, ", t_prime(i));
endfor
printf("};\n");
