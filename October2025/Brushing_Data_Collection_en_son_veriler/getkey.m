% getkey - Get the ASCII of the pressed key in a Figure view
% =====================================================================
% purpose : Get a character from the keyboard buffer
% note    : This works only in a Matlab figure which has the focus.
% history :
%  2004-11-01  PL   created
% =====================================================================
function [a] = getkey()

a = double( get(gcbo, 'currentcharacter'));
% disp( [ 'getkey.m : ' num2str(a) ]);

%------------------------  *  *  *  ----------------------------------%
