function x = gen_points(a,b,f,n)
%% this function writes a list of points for testing RANSAC for a line2D
%% output file: line2d.txt
%% usage:       gen_points(a,b,f,n)
%% a = line slope
%% b = line intercept
%% f = fraction of points that are inliers
%% n = total number of points in the set

xl = -10;
xr = 10;
% generate inlier set
nin = floor(f*n);
xin = rand(nin,1)*(xr-xl) + xl;
yin = xin*a + b + 0.5*randn(size(xin));

% generate outlier set
nou = n-nin;
xou = rand(nou,1)*(xr-xl) + xl;
you = rand(nou,1)*(xr-xl) + xl;

x = [xin yin; xou you];

fid = fopen('line2d.txt','w');
fprintf(fid,'%d\n',n);
for j = 1:n
  fprintf(fid,'%1.12e %1.12e\n',x(j,1),x(j,2));
end
fclose(fid);
