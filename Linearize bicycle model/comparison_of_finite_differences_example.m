
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                                     
%   example  for finite differences  from internet 
%   link: http://folk.ntnu.no/alfonsom/MEDT8007/html/problem_1.html       
%                                                                                    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


f = inline('cos(2*pi*x)+sin(4*pi*x+0.25)');
df= inline('-sin(2*pi*x)*2*pi+cos(4*pi*x+0.25)*4*pi');



x0=0.58;


x=linspace(0,1,100);
figure;
plot(x,f(x)); hold on; grid on;
plot(x0,f(x0),'ro');
plot(x0+[-1 1],f(x0)+df(x0)*[-1 1],'r--');
axis([0 1 min(f(x)) max(f(x))]);
xlabel('x','FontSize', 16);
ylabel('f(x)','FontSize', 16);
legend('f(x)','x0','df(x0)','FontSize', 12);
set(gca,'FontSize', 12);


h=logspace(-2,-5,50);

FD=(f(x0+h)-f(x0))./h;       % forward difference
BD=(f(x0)-f(x0-h))./h;       % backward difference
CD=(f(x0+h)-f(x0-h))/2./h;   % central difference


error_FD=abs(FD-df(x0));
error_BD=abs(BD-df(x0));
error_CD=abs(CD-df(x0));


figure;
loglog(h,error_FD,'b.-'); hold on; grid on;
loglog(h,error_BD,'ro--');
loglog(h,error_CD,'gs-');
legend('Forward difference','Backward difference','Central difference','Location','southeast','FontSize', 12);
xlabel('h','FontSize', 16);
ylabel('Absolute Error','FontSize', 16);
set(gca,'FontSize', 12);
