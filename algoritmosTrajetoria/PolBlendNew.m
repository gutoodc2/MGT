clc; clear all; close all;

delete(instrfindall);
x=serial('COM3','BAUD', 9600);

vel_max_motor = 22;
vet_velocidades = 0:1:vel_max_motor;

for i=1:vel_max_motor+1
    vet_pwm(i) = round((vet_velocidades(i) - 0)*(255-0)/(vel_max_motor-0) + 0);
end

tf_l = 6;
w_l  = 10;
posi = 30;
posf = 70;

d_theta=(posf-posi);

if d_theta>0
    d_theta=-d_theta;
    u=1;
else
    u=-1;
end

tb=(d_theta+(w_l*tf_l))/w_l

ta=tf_l/100;
c2=w_l/tb;

for t=0:99
    if (t*ta)<=tb
        theta_t(t+1)=(posi)+u*(0.5*c2*(t*ta)^2);
        v_t(t+1)=u*c2*t*ta;
        a_t(t+1)=u*c2;
    end
    if (t*ta)>tb && (t*ta)<= (tf_l-tb)
        theta_t(t+1)=(((posi)+u*(0.5*c2*(tb)^2)))+u*(w_l*((t*ta)-tb));
        v_t(t+1)=u*w_l;
        a_t(t+1)=u*0;
    end
    if (t*ta)>(tf_l-tb)
        theta_t(t+1)=(posf)-u*((w_l/(2*tb))*((tf_l-t*ta)^2));
        v_t(t+1)=u*(w_l/tb)*(tf_l-(t*ta));
        a_t(t+1)=-u*w_l/tb;
    end
end

g=linspace(0,tf_l,100);
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
fwrite(x,tf_l)
pause(2.0)
max(v_t)
fwrite(x,max(v_t))
pause(2.0)
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
