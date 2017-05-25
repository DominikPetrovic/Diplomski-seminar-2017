figure(1)
plot(body_pos.time,body_pos.signals.values(:,1))
hold on
grid on
plot(pred_body_pos.time,pred_body_pos.signals.values(:,1))
xlabel('time');
legend('Body position X','Predicted body position X')

figure(2)
plot(body_pos.time,body_pos.signals.values(:,2))
hold on
grid on
plot(pred_body_pos.time,pred_body_pos.signals.values(:,2))
xlabel('time');
legend('Body position Y','Predicted body position Y')

figure(3)
plot(body_pos.time,body_pos.signals.values(:,3))
hold on
grid on
plot(pred_body_pos.time,pred_body_pos.signals.values(:,3))
xlabel('time');
legend('Body position Z','Predicted body position Z')

figure(4)
plot(body_rot.time,body_rot.signals.values(:,1))
hold on
grid on
plot(pred_body_rot.time,pred_body_rot.signals.values(:,1))
xlabel('time');
legend('Body rotation 1','Predicted body rotation 1')

figure(5)
plot(body_rot.time,body_rot.signals.values(:,2))
hold on
grid on
plot(pred_body_rot.time,pred_body_rot.signals.values(:,2))
xlabel('time');
legend('Body rotation 2','Predicted body rotation 2')

figure(6)
plot(body_rot.time,body_rot.signals.values(:,3))
hold on
grid on
plot(pred_body_rot.time,pred_body_rot.signals.values(:,3))
xlabel('time');
legend('Body rotation 3','Predicted body rotation 3')