function b=altlatex(a,opts)
% str=altlatex(sym)
% modifies the latex function to do some expression substitution
% latex now handles 
% syms a12 a_12
    
% opts.splitpts (could) inserts '\\' to allow long equations with the latex
% split or aligned environments (see insert after)
if strcmp(class(a),'sym')
    b=latex(a);
elseif strcmp(class(a),'cell')
    b='\left(';
    for jj=1:length(a)-1;
        b=[b latex(a{jj}) ','];
    end
    b=[b latex(a{end}) '\right)'];
else
    b=latex(sym(a));
end

% Greeks
% b=strrep(b,'\mathrm{alpha}','{\alpha}');
% b=strrep(b,'\mathrm{beta}','{\beta}');
% b=strrep(b,'\mathrm{gamma}','{\gamma}');
% delta
% epsilon
% b=strrep(b,'\mathrm{zeta}','{\zeta}');
% eta
% b=strrep(b,'\mathrm{theta}','{\theta}');b=strrep(b,'\mathrm{vartheta}','{\vartheta}');
% iota
% kappa
% b=strrep(b,'\mathrm{lambda}','{\lambda}');
% b=strrep(b,'\mathrm{mu}','{\mu}');
% b=strrep(b,'\mathrm{nu}','{\nu}');
%xi
%omicron
%pi
%rho
%sigma
% b=strrep(b,'\mathrm{tau}','{\tau}');
%upsilon
% b=strrep(b,'\mathrm{phi}','{\phi}');
%chi
%psi
% b=strrep(b,'\mathrm{omega}','{\omega}');

% non greeks
b=strrep(b,'\mathrm{wn}_{1}','\omega_{n,1}');
b=strrep(b,'\mathrm{wn}_{2}','\omega_{n,2}');
b=strrep(b,'\mathrm{wn}_{3}','\omega_{n,3}');

b=strrep(b,'\mathrm{wn}','{w_n}');

%simple sin/cos for theta 1-3
b=strrep(b,'\cos\left(\theta _{1}\right)','\cos\theta_{1}');
b=strrep(b,'\cos\left(\theta _{2}\right)','\cos\theta_{2}');
b=strrep(b,'\cos\left(\theta _{3}\right)','\cos\theta_{3}');
b=strrep(b,'\sin\left(\theta _{1}\right)','\sin\theta_{1}');
b=strrep(b,'\sin\left(\theta _{2}\right)','\sin\theta_{2}');
b=strrep(b,'\sin\left(\theta _{3}\right)','\sin\theta_{3}');


% differentials
b=strrep(b,'\mathrm{dtheta}','{\dot\theta}');
b=strrep(b,'\mathrm{ddtheta}','{\ddot\theta}');
b=strrep(b,'\mathrm{dphi}','{\dot\phi}');
b=strrep(b,'\mathrm{ddphi}','{\ddot\phi}');
b=strrep(b,'\mathrm{domega}','{\dot\omega}');

b=strrep(b,'\mathrm{dotq}_','{\dot{q}_');
b=strrep(b,'\mathrm{ddotq}_','{\ddot{q}_');

% annoying spaces
b=strrep(b,'\,',' ');

% dubious subs that perhaps should be avoided
%b=strrep(b,'\mathrm{qq}','{\theta}');
%b=strrep(b,'\mathrm{dq}','{\dot\theta}');

% matrix hack
b=strrep(b,'\left(\begin{array}{cccccccc}','\begin{bmatrix}');
b=strrep(b,'\left(\begin{array}{ccccccc}','\begin{bmatrix}');
b=strrep(b,'\left(\begin{array}{cccccc}','\begin{bmatrix}');
b=strrep(b,'\left(\begin{array}{ccccc}','\begin{bmatrix}');
b=strrep(b,'\left(\begin{array}{cccc}','\begin{bmatrix}');
b=strrep(b,'\left(\begin{array}{ccc}','\begin{bmatrix}');
b=strrep(b,'\left(\begin{array}{cc}','\begin{bmatrix}');
b=strrep(b,'\left(\begin{array}{c}','\begin{bmatrix}');
b=strrep(b,'\end{array}\right)','\end{bmatrix}');

% double subscript on a hack
% latex.m now handles numeric subscripts on single characters ie
% a_1, hence this is now a double character hack
b=strrep(b,'\mathrm{aa}_{11}','a_{1,1}');
b=strrep(b,'\mathrm{aa}_{12}','a_{1,2}');
b=strrep(b,'\mathrm{a13}','a_{1,3}');
b=strrep(b,'\mathrm{a14}','a_{1,4}');
b=strrep(b,'\mathrm{a21}','a_{2,1}');
b=strrep(b,'\mathrm{a22}','a_{2,2}');
b=strrep(b,'\mathrm{a23}','a_{2,3}');
b=strrep(b,'\mathrm{a24}','a_{2,4}');
b=strrep(b,'\mathrm{a31}','a_{3,1}');
b=strrep(b,'\mathrm{a32}','a_{3,2}');
b=strrep(b,'\mathrm{a33}','a_{3,3}');
b=strrep(b,'\mathrm{a34}','a_{3,4}');
b=strrep(b,'\mathrm{a41}','a_{4,1}');
b=strrep(b,'\mathrm{a42}','a_{4,2}');
b=strrep(b,'\mathrm{a43}','a_{4,3}');
b=strrep(b,'\mathrm{a44}','a_{4,4}');
b=strrep(b,'\mathrm{w11}','w_{1,1}');
b=strrep(b,'\mathrm{w12}','w_{1,2}');
b=strrep(b,'\mathrm{w13}','w_{1,3}');
b=strrep(b,'\mathrm{w14}','w_{1,4}');
b=strrep(b,'\mathrm{w21}','w_{2,1}');
b=strrep(b,'\mathrm{w22}','w_{2,2}');

b=strrep(b,'\mathrm{ca1}','c_{a1}');
b=strrep(b,'\mathrm{ca2}','c_{a2}');
b=strrep(b,'\mathrm{cb1}','c_{b1}');
b=strrep(b,'\mathrm{cb2}','c_{b2}');

% specific for Manupulator Dynamics
b=strrep(b,'\mathrm{Ixx2}','I_{xx2}');
b=strrep(b,'\mathrm{Iyy2}','I_{yy2}');
b=strrep(b,'\mathrm{Izz1}','I_{zz1}');
b=strrep(b,'\mathrm{Izz2}','I_{zz2}');
b=strrep(b,'\mathrm{Izz3}','I_{zz3}');
b=strrep(b,'\mathrm{lcom}_{1}','l_{com1}');
b=strrep(b,'\mathrm{lcom}_{2}','l_{com2}');
b=strrep(b,'\mathrm{lcom}_{3}','l_{com3}');
%b=strrep(b,'\mathrm{lcom1}','l_{com1}');
%b=strrep(b,'\mathrm{lcom2}','l_{com2}');
%b=strrep(b,'\mathrm{lcom3}','l_{com3}');


% other special variables
b=strrep(b,'\mathrm{Bom}','(\frac{B}{m})'); % damping over mass
b=strrep(b,'\mathrm{Kom}','(\frac{K}{m})'); % stiffness over mass
b=strrep(b,'\mathrm{Fom}','(\frac{F}{m})'); % force over mass



end

