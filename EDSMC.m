clc; clear; close all;

Ts = 0.033; % Sampling time
t = 0:Ts:30; % Simulation time
q = [1.1; 0.8; 0]; % Initial robot pose

% Reference
freq = 2*pi/30;
xRef = 1.1 + 0.7*sin(freq*t);
yRef = 0.9 + 0.7*sin(2*freq*t);
dxRef = freq*0.7*cos(freq*t);
dyRef = 2*freq*0.7*cos(2*freq*t);
ddxRef = -freq^2*0.7*sin(freq*t);
ddyRef = -4*freq^2*0.7*sin(2*freq*t);

qRef = [xRef; yRef; atan2(dyRef, dxRef)];
vRef = sqrt(dxRef.^2+dyRef.^2);
wRef = (dxRef.*ddyRef-dyRef.*ddxRef)./(dxRef.^2+dyRef.^2);
uRef = [vRef; wRef];

hAni = OFig(); axis equal; axis([-0.1, 2.3, 0, 1.8]);
xlabel('$x~[\mathrm{m}]$'); ylabel('$y~[\mathrm{m}]$');

% fig = gcf;  % <<<<<< DO NOT CHANGE
% % High-quality rendering without resizing
% set(fig,'Color','w','Renderer','opengl','GraphicsSmoothing','on');
% set(gca,'FontSize',14);

ref = OWmr(); ref.showPath('r--', true);
% for k = 1:length(t)
%     ref.setPose(qRef(:,k));
% end

wmr = OWmr(); wmr.showPath('b-', true); wmr.setPose(q); wmr.showWmr('y-', 0.2, 0.04, 0.009);
% legend([wmr.oPath() ref.oPath()],{'Actual','desired'});
sigU = OSig(2); sigU.axes('$t~[\mathrm{s}]$', '$v$', '$\omega$'); sigU.sig('r--', 'b-');
sige = OSig(3); sige.axes('$t~[\mathrm{s}]$', '$x_e$', '$y_e$', '$\theta_e$'); sige.sig('r--', 'b-');

% gifFile  = 'wmr_event_HQ_same_size.gif';
% gifDelay = Ts;
% gifStep  = 2;   % save every 2nd frame (optional)

% ===== EVENT-TRIGGER ADDITIONS =====
uhatB = [0;0];     % held DSMC input
sigma = 0.75;      % trigger parameter
event = 0;
event_times  = [];
event_index  = [];

% ==================================

for k = 1:length(t)

    e = [cos(q(3)), sin(q(3)), 0; ...
        -sin(q(3)), cos(q(3)), 0; ...
         0,         0,         1]*(qRef(:,k) - q);
    e(3) = wrapToPi(e(3));

    A = [1, Ts*uRef(2,k),   0;
        -Ts*uRef(2,k),   1, Ts*uRef(1,k);
         0, 0, 1];

    B = [-Ts, 0;
          0, 0;
          0, -Ts];

    G = [5 0 0;
         0 25 5];

    S = G*e;

    ep = [.5 0; 0 .5];
    qs = [3 0; 0 3];

    % ===== ORIGINAL DSMC LAW =====
    uB = -(G*B)\(G*A*e - S + qs*Ts*S + ep*Ts*tanh(S));
    % =============================

    % ===== EVENT TRIGGER =====
    if norm(uhatB - uB,2) >= sigma*norm(S,2)
        uhatB = uB;
        event = event + 1,
        event_times(end+1) = t(k);
        event_index(end+1) = k;
    end
    % =========================

    ur = [uRef(1,k)*cos(e(3)); uRef(2,k)];
    u = uhatB + ur;

    % ===== ORIGINAL PLOTTING =====
    wmr.setPose(q);
    ref.setPose(qRef(:,k));
    sige.plot(t(k), [0, 0, 0], e);
    sigU.plot(t(k), ur, u);
    OFig.pause(Ts);
    % =============================
    
%     %% -------- HIGH-QUALITY GIF WRITE --------
%     if mod(k,gifStep)==0 || k==1
%         frame = getframe(fig);     % capture OFig figure
%         im = im2uint8(frame2im(frame));
%         [imind,cm] = rgb2ind(im,256,'nodither');
% 
%         if k == 1
%             imwrite(imind,cm,gifFile,'gif', ...
%                 'Loopcount',inf,'DelayTime',gifDelay);
%         else
%             imwrite(imind,cm,gifFile,'gif', ...
%                 'WriteMode','append','DelayTime',gifDelay);
%         end
%     end

    dq = [u(1)*cos(q(3));
          u(1)*sin(q(3));
          u(2)];

    q = q + Ts*dq;
    q(3) = wrapToPi(q(3));
end


OFig.pause(Ts);
sigU.fin();

%% =========================================================

figure;
dt_event = diff(event_times);
stem(event_times(2:end), dt_event,'LineWidth',1.5);
xlabel('$t$ (s)','Interpreter','latex');
ylabel('$\Delta t_k$ (s)','Interpreter','latex');
grid on;

disp(['Total Events = ', num2str(event)])


