%% Does function evaluation given state values
% s containts (dq1, dq2, dq3, dq4, q1, q2, q3, q4, u1, u2, u3, u4) of that particular instant
function [fval]=evaldyn(s)
% s = [x,w]
% x = [dq, q]
qv1 = s(1);
qv2 = s(2);
qv3 = s(3);
qv5 = s(4);
qv6 = s(5);
qv7 = s(6);

u1 = s(7);
u2 = s(8);
u3 = s(9);

c5=cos(qv5);c6=cos(qv6);c7=cos(qv7);s5=sin(qv5);s6=sin(qv6);s7=sin(qv7);

fval(1) = ((-0.339097E2)+0.261977E2.*s6.^2+c7.^4.*(0.410934E-1+0.410934E-1.* ...
  s6.^2)+c7.^3.*(0.93886E-16+0.916867E0.*s6.^2)+0.186339E0.*s7.^2+ ...
  0.479358E0.*s6.^2.*s7.^2+0.410934E-1.*s7.^4+0.410934E-1.*s6.^2.* ...
  s7.^4+c6.^2.*((-0.208385E2)+0.341389E1.*c7+0.116143E2.*c7.^3+ ...
  0.1E1.*c7.^4+0.187589E1.*s7.^2+(-0.410934E-1).*s7.^4+c7.^2.*( ...
  0.308774E2+(-0.958907E0).*s7.^2))+c7.^2.*((-0.530056E1)+( ...
  -0.821869E-1).*s7.^2+s6.^2.*(0.719198E1+0.246561E0.*s7.^2))+c7.*(( ...
  -0.264198E2)+0.93886E-16.*s7.^2+s6.^2.*(0.231792E2+0.916867E0.* ...
  s7.^2))).^(-1).*(0.E-323+(-0.535915E1).*qv2.*qv3.*s6+(-0.225199E1) ...
  .*c7.*qv2.*qv3.*s6+0.416195E1.*c7.^2.*qv2.*qv3.*s6+0.183373E1.* ...
  c7.^3.*qv2.*qv3.*s6+0.164374E0.*c7.^4.*qv2.*qv3.*s6+(-0.121705E2) ...
  .*qv1.*qv3.*s7+(-0.750412E0).*c7.*qv1.*qv3.*s7+0.13753E1.*c7.^2.* ...
  qv1.*qv3.*s7+0.164374E0.*c7.^3.*qv1.*qv3.*s7+0.477572E3.*s5.*s7+ ...
  0.200682E3.*c7.*s5.*s7+0.179889E2.*c7.^2.*s5.*s7+0.110445E2.*qv1.* ...
  qv3.*s6.^2.*s7+0.396008E1.*c7.*qv1.*qv3.*s6.^2.*s7+(-0.458433E0).* ...
  c7.^2.*qv1.*qv3.*s6.^2.*s7+(-0.164374E0).*c7.^3.*qv1.*qv3.*s6.^2.* ...
  s7+(-0.526151E3).*s5.*s6.^2.*s7+(-0.288995E3).*c7.*s5.*s6.^2.*s7+( ...
  -0.359778E2).*c7.^2.*s5.*s6.^2.*s7+0.952278E0.*qv2.*qv3.*s6.* ...
  s7.^2+0.183373E1.*c7.*qv2.*qv3.*s6.*s7.^2+0.328748E0.*c7.^2.*qv2.* ...
  qv3.*s6.*s7.^2+0.458433E0.*qv1.*qv3.*s7.^3+(-0.164374E0).*c7.* ...
  qv1.*qv3.*s7.^3+(-0.179889E2).*s5.*s7.^3+0.458433E0.*qv1.*qv3.* ...
  s6.^2.*s7.^3+0.164374E0.*c7.*qv1.*qv3.*s6.^2.*s7.^3+0.164374E0.* ...
  qv2.*qv3.*s6.*s7.^4+c5.*c6.*((-0.25042E4)+(-0.10523E4).*c7+ ...
  0.455745E3.*c7.^3+0.408525E2.*c7.^4+0.943271E2.*s7.^2+c7.^2.*( ...
  0.990229E3+(-0.408525E2).*s7.^2))+c6.^3.*qv1.^2.*s7.*(( ...
  -0.608526E1)+(-0.268492E1).*c7.^3+(-0.186645E0).*c7.^4+ ...
  0.229217E0.*s7.^2+c7.^2.*((-0.119083E2)+0.186645E0.*s7.^2)+c7.*(( ...
  -0.185585E2)+0.602734E0.*s7.^2))+c6.^2.*s7.*(qv1.*qv3.*(( ...
  -0.121705E2)+(-0.229217E1).*c7.^2+(-0.164374E0).*c7.^3+ ...
  0.458433E0.*s7.^2+c7.*((-0.947803E1)+0.164374E0.*s7.^2))+s5.*(( ...
  -0.477572E3)+(-0.473734E3).*c7.^2+(-0.408525E2).*c7.^3+ ...
  0.179889E2.*s7.^2+c7.*((-0.128524E4)+0.408525E2.*s7.^2)))+( ...
  -0.633468E3).*u1+(-0.266192E3).*c7.*u1+(-0.238612E2).*c7.^2.*u1+ ...
  0.238612E2.*s7.^2.*u1+0.133096E3.*s6.*s7.*u2+0.477223E2.*c7.*s6.* ...
  s7.*u2+c6.*((-0.37329E0).*c7.^4.*qv2.^2.*s7+(-0.121705E2).* ...
  qv3.^2.*s7+0.458433E0.*qv3.^2.*s7.^3+qv1.*qv2.*s6.*((-0.994316E2)+ ...
  0.624656E1.*c7.^4+0.37329E0.*c7.^5+0.374534E1.*s7.^2+c7.^2.*( ...
  0.464185E2+(-0.624656E1).*s7.^2)+c7.^3.*(0.326803E2+(-0.74658E0).* ...
  s7.^2)+c7.*((-0.53953E2)+(-0.94517E1).*s7.^2+0.37329E0.*s7.^4))+ ...
  qv1.^2.*s7.*((-0.608526E1)+c7.^4.*(0.186645E0+0.186645E0.*s6.^2)+ ...
  c7.^3.*(0.164383E1+0.252055E1.*s6.^2)+0.229217E0.*s7.^2+s6.^2.*(( ...
  -0.492336E1)+0.229217E0.*s7.^2)+c7.*((-0.141947E2)+0.43836E0.* ...
  s7.^2+s6.^2.*(0.949711E1+(-0.43836E0).*s7.^2))+c7.^2.*(( ...
  -0.164436E0)+(-0.186645E0).*s7.^2+s6.^2.*(0.961614E1+(-0.186645E0) ...
  .*s7.^2)))+0.633468E3.*u3+(-0.238612E2).*s7.^2.*u3+c7.^3.*(( ...
  -0.520547E1).*qv2.^2.*s7+0.541883E2.*u3)+c7.^2.*((-0.458433E0).* ...
  qv3.^2.*s7+qv2.^2.*((-0.215244E2).*s7+0.37329E0.*s7.^3)+ ...
  0.628379E3.*u3)+c7.*((-0.511422E1).*qv3.^2.*s7+qv2.^2.*(( ...
  -0.27639E2).*s7+0.104109E1.*s7.^3)+(0.170479E4+(-0.541883E2).* ...
  s7.^2).*u3)));



fval(2) = ((-0.825184E3)+0.637515E3.*s6.^2+c7.^4.*(0.1E1+0.1E1.*s6.^2)+ ...
  c7.^3.*(0.22847E-14+0.223117E2.*s6.^2)+0.453451E1.*s7.^2+ ...
  0.116651E2.*s6.^2.*s7.^2+0.1E1.*s7.^4+0.1E1.*s6.^2.*s7.^4+c6.^2.*( ...
  (-0.507101E3)+0.830764E2.*c7+0.282632E3.*c7.^3+0.243348E2.*c7.^4+ ...
  0.456493E2.*s7.^2+(-0.1E1).*s7.^4+c7.^2.*(0.751394E3+(-0.233348E2) ...
  .*s7.^2))+c7.^2.*((-0.128988E3)+(-0.2E1).*s7.^2+s6.^2.*( ...
  0.175015E3+0.6E1.*s7.^2))+c7.*((-0.642921E3)+0.22847E-14.*s7.^2+ ...
  s6.^2.*(0.56406E3+0.223117E2.*s7.^2))).^(-1).*(0.E-323+0.35674E5.* ...
  s5.*s6+0.196071E5.*c7.*s5.*s6+0.129406E4.*c7.^2.*s5.*s6+( ...
  -0.218878E3).*c7.^3.*s5.*s6+(-0.275607E5).*s5.*s6.^3+(-0.180598E5) ...
  .*c7.*s5.*s6.^3+(-0.358949E4).*c7.^2.*s5.*s6.^3+(-0.218878E3).* ...
  c7.^3.*s5.*s6.^3+(-0.346754E3).*qv2.*qv3.*s7+(-0.248784E3).*c7.* ...
  qv2.*qv3.*s7+(-0.334676E2).*c7.^2.*qv2.*qv3.*s7+0.4E1.*c7.^3.* ...
  qv2.*qv3.*s7+0.295293E3.*qv2.*qv3.*s6.^2.*s7+0.230333E3.*c7.*qv2.* ...
  qv3.*s6.^2.*s7+0.334676E2.*c7.^2.*qv2.*qv3.*s6.^2.*s7+(-0.4E1).* ...
  c7.^3.*qv2.*qv3.*s6.^2.*s7+(-0.129406E4).*s5.*s6.*s7.^2+( ...
  -0.656635E3).*c7.*s5.*s6.*s7.^2+0.114771E4.*s5.*s6.^3.*s7.^2+ ...
  0.218878E3.*c7.*s5.*s6.^3.*s7.^2+(-0.111559E2).*qv2.*qv3.*s7.^3+( ...
  -0.4E1).*c7.*qv2.*qv3.*s7.^3+0.111559E2.*qv2.*qv3.*s6.^2.*s7.^3+ ...
  0.4E1.*c7.*qv2.*qv3.*s6.^2.*s7.^3+c6.^3.*qv1.^2.*s6.*(0.458689E3+( ...
  -0.243348E2).*c7.^4+(-0.120014E2).*s7.^2+0.1E1.*s7.^4+c7.^3.*(( ...
  -0.282632E3)+0.908393E1.*s7.^2)+c7.^2.*((-0.689717E3)+0.780043E2.* ...
  s7.^2)+c7.*((-0.548016E2)+0.929696E2.*s7.^2))+qv1.*qv3.*s6.*( ...
  0.763444E2+c7.^3.*(0.111559E2+(-0.334676E2).*s6.^2)+c7.^4.*(( ...
  -0.2E1)+(-0.2E1).*s6.^2)+0.251771E1.*s7.^2+(-0.2E1).*s7.^4+s6.^2.* ...
  ((-0.589816E2)+(-0.117434E2).*s7.^2+(-0.2E1).*s7.^4)+c7.*( ...
  0.374154E3+(-0.111559E2).*s7.^2+s6.^2.*((-0.295293E3)+( ...
  -0.111559E2).*s7.^2))+c7.^2.*(0.184163E3+(-0.4E1).*s7.^2+s6.^2.*(( ...
  -0.174937E3)+(-0.4E1).*s7.^2)))+0.323886E4.*s6.*s7.*u1+ ...
  0.116131E4.*c7.*s6.*s7.*u1+(-0.902415E4).*u2+(-0.323886E4).*c7.* ...
  u2+0.290328E3.*c7.^2.*u2+0.697182E4.*s6.^2.*u2+0.323886E4.*c7.* ...
  s6.^2.*u2+0.290328E3.*c7.^2.*s6.^2.*u2+(-0.290328E3).*s7.^2.*u2+( ...
  -0.290328E3).*s6.^2.*s7.^2.*u2+c6.^2.*((-0.213091E3).*qv2.*qv3.* ...
  s7+0.480486E2.*c7.*qv2.*qv3.*s7+0.316099E3.*c7.^2.*qv2.*qv3.*s7+ ...
  0.973391E2.*c7.^3.*qv2.*qv3.*s7+0.111559E2.*qv2.*qv3.*s7.^3+ ...
  0.4E1.*c7.*qv2.*qv3.*s7.^3+qv1.*qv3.*s6.*(0.46916E2+(-0.293787E3) ...
  .*c7.^3+(-0.486696E2).*c7.^4+0.215681E2.*s7.^2+0.2E1.*s7.^4+ ...
  c7.^2.*((-0.146022E3)+0.546696E2.*s7.^2)+c7.*(0.18569E3+ ...
  0.557794E2.*s7.^2))+s5.*s6.*(0.219227E5+(-0.532635E4).*c7.^3+ ...
  0.129406E4.*s7.^2+c7.^2.*((-0.303712E5)+0.198827E4.*s7.^2)+c7.*(( ...
  -0.862292E4)+0.620187E4.*s7.^2))+(-0.554562E4).*u2+0.323886E4.* ...
  c7.*u2+0.706507E4.*c7.^2.*u2+0.290328E3.*s7.^2.*u2)+c6.*s6.*(qv1.* ...
  qv2.*s6.*s7.*(0.508384E3+(-0.152009E3).*c7.^3+(-0.181679E2).* ...
  c7.^4+c7.^2.*((-0.26032E3)+0.181679E2.*s7.^2)+c7.*(0.244511E3+ ...
  0.506696E2.*s7.^2))+qv1.^2.*(0.746405E3+c7.^4.*((-1.E0)+(-1.E0).* ...
  s6.^2)+0.240443E2.*s7.^2+(-1.E0).*s7.^4+s6.^2.*((-0.576652E3)+ ...
  0.169138E2.*s7.^2+(-1.E0).*s7.^4)+c7.*(0.614646E3+0.706579E2.* ...
  s7.^2+s6.^2.*((-0.535785E3)+(-0.706579E2).*s7.^2))+c7.^2.*( ...
  0.131523E3+(-0.2E1).*s7.^2+s6.^2.*((-0.172481E3)+(-0.526696E2).* ...
  s7.^2))+c7.^3.*((-0.908393E1).*s7.^2+s6.^2.*((-0.223117E2)+( ...
  -0.908393E1).*s7.^2)))+s7.*(c5.*(0.128038E5+0.459086E4.*c7+( ...
  -0.554523E4).*c7.^2+(-0.198827E4).*c7.^3)+0.181679E2.*c7.^3.* ...
  qv2.^2.*s7+0.622267E2.*qv3.^2.*s7+c7.*(0.141316E3.*qv2.^2.*s7+ ...
  0.223117E2.*qv3.^2.*s7+(-0.851671E4).*u3)+c7.^2.*(0.101339E3.* ...
  qv2.^2.*s7+(-0.263732E4).*u3)+(-0.323886E4).*u3)));


fval(3) = ((-0.40147E-6).*(0.278897E1.*s6.*s7+0.1E1.*c7.*s6.*s7).^2+( ...
  -0.127139E-5).*c6.^2.*(0.440338E0+0.1E1.*c7).^2.*(0.265481E2+ ...
  0.111559E2.*c7+0.1E1.*c7.^2+(-0.1E1).*s7.^2)+0.501837E-7.*( ...
  0.265481E2+0.111559E2.*c7+0.1E1.*c7.^2+(-0.1E1).*s7.^2).*( ...
  0.310826E2+(-0.240136E2).*s6.^2+c7.*(0.111559E2+(-0.111559E2).* ...
  s6.^2)+c7.^2.*((-0.1E1)+(-0.1E1).*s6.^2)+0.1E1.*s7.^2+0.1E1.* ...
  s6.^2.*s7.^2+c6.^2.*(0.240136E2+0.111559E2.*c7+0.1E1.*c7.^2+( ...
  -0.1E1).*s7.^2))).^(-1).*((-0.455865E-6).*c6.*(0.440338E0+0.1E1.* ...
  c7).*(0.265481E2+0.111559E2.*c7+0.1E1.*c7.^2+(-0.1E1).*s7.^2).*( ...
  c5.*c6.*(0.573857E3+0.109439E3.*c7)+0.122809E1.*qv2.*qv3.*s6+( ...
  -1.E0).*c7.^2.*qv2.*qv3.*s6+0.278897E1.*qv1.*qv3.*s7+(-1.E0).*c7.* ...
  qv1.*qv3.*s7+c6.^2.*(0.278897E1+0.1E1.*c7).*qv1.*qv3.*s7+( ...
  -0.109439E3).*s5.*s7+(-0.278897E1).*qv1.*qv3.*s6.^2.*s7+(-1.E0).* ...
  c7.*qv1.*qv3.*s6.^2.*s7+0.1E1.*qv2.*qv3.*s6.*s7.^2+c6.*((( ...
  -0.278897E1)+(-1.E0).*c7).*qv2.^2.*s7+0.278897E1.*qv3.^2.*s7+qv1.* ...
  qv2.*s6.*(0.240136E2+0.111559E2.*c7+0.1E1.*c7.^2+(-1.E0).*s7.^2))+ ...
  0.145164E3.*u1)+(-0.455865E-6).*c6.*(0.440338E0+0.1E1.*c7).*( ...
  0.278897E1+0.1E1.*c7).*s6.*s7.*(0.114771E4.*s5.*s6+0.218878E3.* ...
  c7.*s5.*s6+(-0.111559E2).*qv2.*qv3.*s7+(-0.4E1).*c7.*qv2.*qv3.*s7+ ...
  qv1.*qv3.*s6.*(0.245618E1+0.111559E2.*c7+0.2E1.*c7.^2+(-0.2E1).* ...
  s7.^2)+c6.*qv1.^2.*s6.*(0.240136E2+0.111559E2.*c7+0.1E1.*c7.^2+( ...
  -0.1E1).*s7.^2)+(-0.290328E3).*u2)+(-1).*((-0.47455E-4).*( ...
  0.278897E1.*s6.*s7+0.1E1.*c7.*s6.*s7).^2+(0.914417E-1+ ...
  0.384251E-1.*c7+0.344438E-2.*c7.^2+(-0.344438E-2).*s7.^2).*( ...
  0.535301E-1+(-0.413559E-1).*s6.^2+c7.*(0.192125E-1+(-0.192125E-1) ...
  .*s6.^2)+c7.^2.*((-0.172219E-2)+(-0.172219E-2).*s6.^2)+ ...
  0.172219E-2.*s7.^2+0.172219E-2.*s6.^2.*s7.^2+c6.^2.*(0.413559E-1+ ...
  0.192125E-1.*c7+0.172219E-2.*c7.^2+(-0.172219E-2).*s7.^2))).*(( ...
  -0.7539E0).*c5.*c7+0.192125E-1.*qv2.^2.*s7+0.688876E-2.*c7.* ...
  qv2.^2.*s7+0.7539E0.*c6.*s5.*s7+qv1.^2.*(0.960626E-2+c6.^2.*( ...
  0.960626E-2+0.344438E-2.*c7)+(-0.344438E-2).*c7+(-0.960626E-2).* ...
  s6.^2+(-0.344438E-2).*c7.*s6.^2).*s7+qv1.*qv2.*s6.*((-0.846001E-2) ...
  +(-0.384251E-1).*c7+(-0.688876E-2).*c7.^2+0.688876E-2.*s7.^2)+( ...
  -0.1E1).*u3));


fval(4) = s(1);
fval(5) = s(2);
fval(6) = s(3);

fval = transpose(fval);
end