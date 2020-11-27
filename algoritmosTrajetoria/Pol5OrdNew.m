clc; clear all; close all;

delete(instrfindall);
x=serial('COM3','BAUD', 9600);

vel_max_motor = 22;
vet_velocidades = 0:1:vel_max_motor;

for i=1:vel_max_motor+1
    vet_pwm(i) = round((vet_velocidades(i) - 0)*(255-0)/(vel_max_motor-0) + 0);
end

tf_5 = 6;
vi_5 = 0;
vf_5 = 0;
ai_5 = 0;
af_5 = 0;

theta= [30; vi_5;ai_5; 70; vf_5;af_5];
a=[1 0 0 0 0 0; 0 1 0 0 0 0;0 0 2 0 0 0;1 tf_5 tf_5^2 tf_5^3 tf_5^4 tf_5^5;0 1 2*tf_5 3*tf_5^2 4*tf_5^3 5*tf_5^4;0 0 2 6*tf_5 12*tf_5^2 20*tf_5^3];
c=inv(a)*theta;
ta=tf_5/100;

for t=0:99
    theta_t(t+1)= c(1)+c(2)*(t*ta)+c(3)*((t*ta)^2)+c(4)*((t*ta)^3)+ (c(5)*(t*ta)^4)+(c(6)*(t*ta)^5);
    v_t(t+1)=c(2)+2*c(3)*(t*ta)+(3*c(4)*(t*ta)^2)+(4*c(5)*(ta*t)^3)+(5*c(6)*(t*ta)^4);
    a_t(t+1)= 2*c(3)+6*c(4)*(t*ta)+(12*c(5)*(t*ta)^2)+(20*c(6)*(t*ta)^3);
end

g=linspace(0,tf_5,100);
subplot(3,1,1);
plot(g,theta_t);
title('Posição');

subplot(3,1,2);
plot(g,v_t);
title('Velocidade');

subplot(3,1,3);
plot(g,a_t);
title('Aceleração');

for i=1:100
    v_t(i)=round(v_t(i));
end

fopen(x);
pause(2.0)
fwrite(x,tf_5)
max(v_t)
fwrite(x,max(v_t))
max(vet_pwm(max(v_t)+1))
fwrite(x,max(vet_pwm(max(v_t)+1)))
for i=1:100
    fwrite(x,v_t(i))
end

pause(2.0)
for i=1:100
    read_Velocity(i) = fread(x,1,'uint8');
    read_Input(i) = fread(x,1,'uint8');
    read_Setpoint(i) = fread(x,1,'uint8');
    read_Output(i) = fread(x,1,'uint8');
end

% pause(2.0)
% for i=1:100
%     read_Setpoint(i) = fread(x,1,'uint8');
% end
fclose(x);

figure(2)
subplot(2,1,1)
plot(g,read_Velocity);
title('Retorno Velocidade encoder');
subplot(2,1,2);
plot(g,v_t);
title('Velocidade Desejada');

figure(3)
plot(g,read_Input,g,read_Setpoint,g,read_Output);
title('Inputs x Setpoints x Outputs');
legend('Inputs','Setpoints','Outputs');
%plot(g,v_t);
%title('Velocidade');

figure(4)
plot(g,v_t,g,read_Velocity);
title('Velocidade Desejada x Velocidade Real');
legend('Velocidade Desejada','Velocidade Real');
