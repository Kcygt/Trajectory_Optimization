function [x, f] = hippodrome(t, x0, loopCounter,x_d_star,completeMatrix_play1,jj)

fq = 1/8;
r = 0.05;
m = 0.02;
x = zeros(3, 1);
if t < 4 && loopCounter == 0
    x = [m*t; 0; 0];
    f = [0; 0; 1.0];
%     f = [0; 0; fe(3)-((fe(3)-1.5)*exp(t-1))];

elseif t < 4 && loopCounter > 0
%     disp(jj)
     x = [m*t; 0; 0]-([0; 0; x_d_star(jj,3)] - [0; 0; completeMatrix_play1(jj+1,5)]);
     f = [0; 0; 1.0];
%      disp(x_d_star(jj,3) - completeMatrix_play1(jj+1,5))
     
elseif 4 <= t && t < 8
        x  = [m*4; 0; 0] + [0; 0; (r*t)/2 - (2*r)];% + [0; 0; 0.05];

%     x  = [m*4; 0; 0] + [0; 0; r*cos(2*pi*fq*t)] + [0; 0; r];
%     x  = [m*4; 0; 0] + [0; 0; r*cos(2*pi*fr*t)] + [0; 0; r];

%     f = [0; 0; 1.5*exp(-1.3*(t-4))];
    f = [0; 0; 0];
    

    
elseif 8 <= t && t < 12
    x = [m*12; 0; 2*r]  - [m*t; 0; 0];

%     x = [m*12; 0; 0] + [-r*sin(2*pi*fq*8); 0; r*cos(2*pi*fq*8)] + [0; 0; r] - [m*t; 0; 0];
    f = [0; 0; 0.0];
    
elseif 12<= t && t <= 16
    
%      x = [0; 0; 2*r] - [0; 0; (r*t)/8 ];
    
    x = [0; 0; r] - [-r*sin(2*pi*fq*t); 0; r*cos(2*pi*fq*t)];

%     x = [-r*sin(2*pi*fq*8); 0; r*cos(2*pi*fq*8)] - [-r*sin(2*pi*fq*t); 0; r*cos(2*pi*fq*t)];
    f = [0; 0; 0.0];
    
end

x = x + x0;

end