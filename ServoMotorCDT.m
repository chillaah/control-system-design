%% EGB345 Servo Motor Control Design Task

%% 2.1 Initial Closed Loop
close all; clear; clc

% estimated parameters from Servo Motor System Identification Task Q.3
km = 7.22;
alpha = 4.46;

% numerator and denominator
numOL = km;
denOL = [1 alpha 0];

% open loop transfer function
Gol = tf(numOL, denOL);

% unit gain controller = 1
K_unit = 1;                   

% cascaded transfer function
CasG = K_unit * Gol;

% unity feedback
H = 1;

% closed loop transfer function
Gcl = feedback(CasG, H);

% plotting step response of the closed loop system
figure();
step(Gcl, 'black');
grid on
box on
xlabel('Time', 'FontSize', 16);
ylabel('Amplitude', 'FontSize', 16);
title('Step Response of Closed Loop System', 'FontSize', 20);


%% 2.2 Design Control For Requested Response

% damping ratio for 5% OS
zeta_new = sqrt(log(5/100)^2)/sqrt(pi^2 + log(5/100)^2);

% gain for % OS = 5%
K_new = 1.4462;

% cascaded transfer function
CasG_new = K_new * Gol;

% closed loop transfer function
Gcl_new = feedback(CasG_new, H);

% plotting step response of the closed loop system
figure();
step(Gcl_new, 'black');
grid on
box on
xlabel('Time', 'FontSize', 16);
ylabel('Amplitude', 'FontSize', 16);
title(['Step Response of Closed Loop System (K = ', num2str(K_new), ')'], 'FontSize', 20);

% natural frequnecy for 5% OS
denCL = cell2mat(Gcl_new.Denominator);
wn_new = denCL(2)/(2 * zeta_new);


%% 2.3 Investigation Of The Gain K

% 10% ranged values from 0.5 to 1.5
vals = 0.5 : 0.1 : 1.5;

% disregarding the value 1 (don't need 1.4462 again)
vals(vals == 1) = [];

% modified gain values
modgain = vals .* K_new;

% initializing cascaded and closed loop transfer functions
CasG_diff = tf(1,10);
Gcl_diff = tf(1,10);

% for loop to calculate all cascaded transfer functions
% and plot all the step responses in order
for i = 1:length(modgain)
    CasG_diff(i) = modgain(i) * Gol;
    Gcl_diff(i) = feedback(CasG_diff(i), H);
    figure();
    step(Gcl_diff(i), 'black');
    grid on
    box on
    xlabel('Time', 'FontSize', 16);
    ylabel('Amplitude', 'FontSize', 16);
    title(['Step Response With Gain K = ', num2str(modgain(i))], 'FontSize', 16);
end


%% 2.4 Improved Transient Response Using A Dynamic Compensator

% --------------------- SIMPLE GAIN COMPENSATION --------------------------

% checking if simple gain compensation can halve the settling time Ts
figure();
rlocus(CasG_new);
grid on
box on
title(['Root Locus of Cascade System With Gain K = ', num2str(K_new)], 'FontSize', 16);

% -------------- DYNAMIC COMPENSATION (LEAD CONTROLLER) -------------------

% introducing a zero for pole-zero cancellation with pole at s = -4.46
Zero_C = [1 4.46];

% introducing a pole twice as far away from the previous pole (2 * 4.46)
% to halve settling time
Pole_C = [1 8.92];

% transfer function of lead compensator without gain
G_C = tf(Zero_C, Pole_C);

% foward transfer funtion of system without system gain
G = G_C * Gol;

% root locus of system with lead compensator without system gain
figure();
rlocus(G)
grid on
box on
title('Root Locus of Cascade System With Lead Compensator', 'FontSize', 16);

% overall gain of the system (new gain)
K = 5.78;

% new cascade system with lead compensator and new gain
CasG_C = K * G;

% closed loop system with halved settling time
% unity feedback
Gcl_C = feedback(CasG_C, H);

% step response of closed loop system
figure();
step(Gcl_C, 'black');
grid on
box on
xlabel('Time', 'FontSize', 16);
ylabel('Amplitude', 'FontSize', 16);
title(['Step Response of Closed Loop System With Lead Compensator & K = ', num2str(K), ], 'FontSize', 20);


%% 2.5 Robustness Study 1 - Model Parameter Error

% ------------------------- SAME VALUE CASES ------------------------------

% model parameters
km_low = 0.9 * km;
km_high = 1.1 * km;
alpha_low = 0.9 * alpha;
alpha_high = 1.1 * alpha;

numlow = km_low;
numhigh = km_high;
denlow = [1 alpha_low 0];
denhigh = [1 alpha_high 0];

% for 0.9*km and 0.9*alpha
Gol_low = tf(numlow, denlow);
wn_low = denlow(2)/(2 * zeta_new);
K_low = wn_low^2 / km_low;
CasG_low = K_low * Gol_low;
Gcl_low = feedback(CasG_low, 1);

figure();
step(Gcl_low, 'r');
hold on
grid on
box on

% for 1.1*km and 1.1*alpha
Gol_high = tf(numhigh, denhigh);
wn_high = denhigh(2)/(2 * zeta_new);
K_high = wn_high^2 / km_high;
CasG_high = K_high * Gol_high;
Gcl_high = feedback(CasG_high, 1);

step(Gcl_high, 'b');
title('Step responses with 5% OS controlled K values');
legend('0.9k and 0.9a', '1.1k and 1.1a');
hold off

% --------------------- DIFFERENT VALUE CASES -----------------------------

% for 0.9*km and 1.1*alpha
Gol_middle1 = tf(numlow, denhigh);
wn_middle1 = denhigh(2)/(2 * zeta_new);
K_middle1 = wn_middle1^2 / km_low;
CasG_middle1 = K_middle1 * Gol_middle1;
Gcl_middle1 = feedback(CasG_middle1, 1);

figure();
step(Gcl_middle1, 'g');
hold on
grid on
box on

% for 1.1*km and 0.9*alpha
Gol_middle2 = tf(numhigh, denlow);
wn_middle2 = denlow(2)/(2 * zeta_new);
K_middle2 = wn_middle2^2 / km_high;
CasG_middle2 = K_middle2 * Gol_middle2;
Gcl_middle2 = feedback(CasG_middle2, 1);

step(Gcl_middle2, 'm');
title('Step responses with 5% OS controlled K values');
legend('0.9k and 1.1a', '1.1k and 0.9a');
hold off

% lowest K value (K = 1.0650) is robust
K_bestdiffkmalpha = K_middle2;

% % confirming with uncertainity
% failcount = 0;
% iter = 1;
% k = 0.9*km : 0.01 : 1.1*km;
% a = 0.9*alpha : 0.01 : 1.1*alpha;
% tot_iter = length(k) * length(a);
% OL = tf(1, tot_iter);
% CASG = tf(1, tot_iter);
% CL = tf(1, tot_iter);
% overshoot = zeros(1,13050);
% tic
% for i = 1:length(k)
%     for j = 1:length(a)
%         OL(iter) = tf(k(i), [1 a(j) 0]);
%         CASG(iter) = K_bestdiffkmalpha * OL(iter);
%         CL(iter) = feedback(CASG(iter),1);
%         overshoot(iter) = stepinfo(CL(iter)).Overshoot;
%         if (overshoot(iter) > 5)
%             failcount = failcount + 1;
%         end
%         iter = iter + 1;
%     end
% end
% toc
% disp('iterated'); disp(iter-1); disp('times');
% disp('incorrect for'); disp(failcount); disp('times');

% ------------------------ JUSTIFY USING ARGUMENT -------------------------

% for 0.9*km and 1.1*alpha
CasG_middle1robust = K_bestdiffkmalpha * Gol_middle1;
Gcl_middle1robust = feedback(CasG_middle1robust, 1);

figure();
step(Gcl_middle1robust, 'r');
hold on
grid on
box on

% for 1.1*km and 0.9*alpha
CasG_middle2robust = K_bestdiffkmalpha * Gol_middle2;
Gcl_middle2robust = feedback(CasG_middle2robust, 1);

step(Gcl_middle2robust, 'b');
xlabel('Time', 'FontSize', 16);
ylabel('Amplitude', 'FontSize', 16);
title('Step Responses For Most Deviated Systems With Robust Gain Value K = 1.0650', 'FontSize', 16);
leg = legend('0.9k_m and 1.1\alpha', '1.1k_m and 0.9\alpha');
set(leg, 'FontSize', 15);
leg = legend('0.9k_m and 1.1\alpha', '1.1k_m and 0.9\alpha');
hold off

% stepinfo(Gcl_middle1robust)
% stepinfo(Gcl_middle2robust)
% disp(stepinfo(Gcl_middle1robust).Overshoot)
% disp(stepinfo(Gcl_middle2robust).Overshoot)

% ----------------- PLOT OF 0.9km 0.9alpha ROBUST SYSTEM ------------------

% for 0.9*km and 0.9*alpha
CasG_lowrobust = K_bestdiffkmalpha * Gol_low;
Gcl_lowrobust = feedback(CasG_lowrobust, 1);

figure();
step(Gcl_lowrobust, 'm');
grid on
box on
xlabel('Time', 'FontSize', 16);
ylabel('Amplitude', 'FontSize', 16);
title('Step Response of 0.9k_m and 0.9\alpha System With Robust Gain Value K = 1.0650', 'FontSize', 20);

% ----------------- PLOT OF 1.1km 1.1alpha ROBUST SYSTEM ------------------

% for 1.1*km and 1.1*alpha
CasG_highrobust = K_bestdiffkmalpha * Gol_high;
Gcl_highrobust = feedback(CasG_highrobust, 1);

figure();
step(Gcl_highrobust, 'g');
grid on
box on
xlabel('Time', 'FontSize', 16);
ylabel('Amplitude', 'FontSize', 16);
title('Step Response of 1.1k_m and 1.1\alpha System With Robust Gain Value K = 1.0650', 'FontSize', 20);


%% 2.6 Robustness Study 2 - Measurement Error

% --------------- HIGHEST AND LOWEST FEEDBACK VALUE CASES -----------------

% measurement parameters
H_low = 0.9 * H;
H_high = 1.1 * H;

% for 0.9*H
K_feedlow = wn_new^2 /(H_low * km);
CasG_feedlow = K_feedlow * Gol;
Gcl_feedlow = feedback(CasG_feedlow, H_low);

figure();
step(Gcl_feedlow, 'r');
hold on
grid on
box on

% for 1.1*H
K_feedhigh = wn_new^2 /(H_high * km);
CasG_feedhigh = K_feedhigh * Gol;
Gcl_feedhigh = feedback(CasG_feedhigh, H_high);

step(Gcl_feedhigh, 'b');
title('Step responses with 5% OS controlled K values');
legend('0.9 feedback', '1.1 feedback');
hold off

% stepinfo(Gcl_feedlow)
% stepinfo(Gcl_feedhigh)

% lowest K value (K = 1.3148)
K_feedback = K_feedhigh;

% h = 0.9 : 0.001 : 1.1; % unity and non unity feedback
% 
% failcount = 0;
% iter = 1;
% tot_iter = length(h);
% Gcl_feedback = tf(1, tot_iter);
% OS_feedback = zeros(1, tot_iter);
% CasG_feedback = K_feedback * Gol;
% for i = 1:length(h)
%     Gcl_feedback(iter) = feedback(CasG_feedback, h(i));
%     OS_feedback(iter) = stepinfo(Gcl_feedback(iter)).Overshoot;
%     if (OS_feedback(iter) > 5)
%         failcount = failcount + 1;
%     end
%     iter = iter + 1;
% end
% disp('iterated'); disp(iter-1); disp('times');
% disp('incorrect for'); disp(failcount); disp('times');

% ------------------------ JUSTIFY USING ARGUMENT -------------------------

% for 0.9*H
CasG_feedlowrobust = K_feedback * Gol;
Gcl_feedlowrobust = feedback(CasG_feedlowrobust, H_low);

figure();
step(Gcl_feedlowrobust, 'r');
hold on
grid on
box on

% for 1.1*H
CasG_feedhighrobust = K_feedback * Gol;
Gcl_feedhighrobust = feedback(CasG_feedhighrobust, H_high);

step(Gcl_feedhighrobust, 'b');
xlabel('Time', 'FontSize', 16);
ylabel('Amplitude', 'FontSize', 16);
title('Step Responses For Most Deviated Systems With Robust Gain Value K = 1.3148', 'FontSize', 16);
leg = legend('0.9H', '1.1H');
set(leg, 'FontSize', 15);
hold off

% stepinfo(Gcl_feedlowrobust)
% stepinfo(Gcl_feedhighrobust)

% ---------------------- PLOT OF 0.9H ROBUST SYSTEM -----------------------

% for 0.9*H
figure();
step(Gcl_feedlowrobust, 'r');
grid on
box on
xlabel('Time', 'FontSize', 16);
ylabel('Amplitude', 'FontSize', 16);
title('Step Response of 0.9H System With Robust Gain Value K = 1.3148', 'FontSize', 16);

% ---------------------- PLOT OF 1.1H ROBUST SYSTEM -----------------------

% for 1.1*H
figure();
step(Gcl_feedhighrobust, 'b');
grid on
box on
xlabel('Time', 'FontSize', 16);
ylabel('Amplitude', 'FontSize', 16);
title('Step Response of 1.1H System With Robust Gain Value K = 1.3148', 'FontSize', 16);


%% 2.7 Robustness Study 3 - An Unmodelled Pole

% --------------------------- BETA = 10*ALPHA -----------------------------

% fast pole beta
beta_faster = 10 * alpha;

num_unmodfaster = km;
den_unmodfaster = [conv([1 alpha],[1 beta_faster]) 0];

Gol_unmodfaster = tf(num_unmodfaster, den_unmodfaster);

CasG_unmodfaster = K_new * Gol_unmodfaster;
Gcl_unmodfaster = feedback(CasG_unmodfaster, 1);

poles_unmodfaster = pole(Gcl_unmodfaster);

stepinfo(Gcl_unmodfaster)
% Ts_unmodfast = stepinfo(Gcl_unmodfaster).SettlingTime;
% OS_unmodfast = stepinfo(Gcl_unmodfaster).Overshoot;
% disp('Ts unmodelled faster pole')
% disp(Ts_unmodfast)
% disp('%OS unmodelled faster pole')
% disp(OS_unmodfast)

figure();
step(Gcl_unmodfaster, 'r')
grid on
box on
xlabel('Time', 'FontSize', 16);
ylabel('Amplitude', 'FontSize', 16);
title('Step Response of System With Pole At \beta = 10\alpha (Faster Pole)', 'FontSize', 16);

% ---------------------------- BETA = 2*ALPHA -----------------------------

% not so fast pole beta
beta_notsofast = 2 * alpha;

num_unmodnotsofast = km;
den_unmodnotsofast = [conv([1 alpha],[1 beta_notsofast]) 0];

Gol_unmodnotsofast = tf(num_unmodnotsofast, den_unmodnotsofast);

CasG_unmodnotsofast = K_new * Gol_unmodnotsofast;
Gcl_unmodnotsofast = feedback(CasG_unmodnotsofast, 1);

poles_unmodnotsofast = pole(Gcl_unmodnotsofast);

stepinfo(Gcl_unmodnotsofast)
% Ts_unmodnotsofast = stepinfo(Gcl_unmodnotsofast).SettlingTime;
% OS_unmodnotsofast = stepinfo(Gcl_unmodnotsofast).Overshoot;
% disp('Ts unmodelled not so fast pole')
% disp(Ts_unmodnotsofast)
% disp('%OS unmodelled not so fast pole')
% disp(OS_unmodnotsofast)

figure();
step(Gcl_unmodnotsofast, 'b')
grid on
box on
xlabel('Time', 'FontSize', 16);
ylabel('Amplitude', 'FontSize', 16);
title('Step Response of System With Pole At \beta = 2\alpha (Not So Fast Pole)', 'FontSize', 16);

% -------------------- IMPACT OF UNMODELLED POLE BETA ---------------------

% beta = 10 * alpha
stepinfo(Gcl_unmodfaster)

% beta = 2 * alpha
stepinfo(Gcl_unmodnotsofast)

% root locus of beta = 10 * alpha
figure();
rlocus(Gol_unmodfaster);
grid on
box on
title('Root Locus of System With Additional Pole At \beta = 10\alpha', 'FontSize', 16);

% root locus of beta = 2 * alpha
figure();
rlocus(Gol_unmodnotsofast);
grid on
box on
title('Root Locus of System With Additional Pole At \beta = 2\alpha', 'FontSize', 16);

