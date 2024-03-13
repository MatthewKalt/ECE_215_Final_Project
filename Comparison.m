Franka_Initial = 22500;
Franka_Hourly = 0.0248;

Human_Hourly = 24;

Human_Array = zeros(2800,1);
Franka_Array = zeros(2800,1);
Franka_Array(1) = Franka_Initial;
Human_Array(1) = Human_Hourly;

for i=2:length(Franka_Array)
Franka_Array(i) = Franka_Array(i-1)+Franka_Hourly;
Human_Array(i) = Human_Hourly*i;
end

k = linspace(1,length(Franka_Array),length(Franka_Array));

plot(k,Human_Array);
hold on
plot(k,Franka_Array);
hold off
ylabel('Cost [$]');
xlabel('Time [Hr]');
legend('Cost for Human Worker','Cost for Robotic Worker',Location='northwest');
title("Human vs Robot Working Cost");


