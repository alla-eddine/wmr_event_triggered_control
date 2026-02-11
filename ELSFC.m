clc; clear; close all;

Ts = 0.033; % Sampling time
t = 0:Ts:30; % Simulation time
q = [1.1; 0.8; 0]; % Initial robot pose
zeta = 0.9;
g = 85;

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

wmr = OWmr(); wmr.showPath('b-', true); wmr.setPose(q); wmr.showWmr('b-', 0.2, 0.04, 0.009);

sigU = OSig(2); sigU.axes('$t~[\mathrm{s}]$', '$v$', '$\omega$'); sigU.sig('r--', 'b-');
sige = OSig(3); sige.axes('$t~[\mathrm{s}]$', '$x_e$', '$y_e$', '$\theta_e$'); sige.sig('r--', 'b-');

% gifFile  = 'wmr_event_HQ_same_size.gif';
% gifDelay = Ts;
% gifStep  = 2;   % save every 2nd frame (optional)

uhatB = [0;0];
event = 0;

% vmax = .6;
% wmax = 1.2;
% Amax = [1 Ts*wmax 0;
%        -Ts*wmax 1 Ts*vmax;
%         0 0 1];
% Bmax = [-Ts 0; 0 0;0 -Ts];
% 
% Kx = 2*zeta*sqrt(wmax^2+g*vmax^2);
% Ky = 2*g*vmax;
% Kmax = [Kx 0 0; 0 Ky Kx];
% 
% Q = .05*eye(3);
% P = dlyap((Amax - Bmax*Kmax)', Q);
% PB_norm = norm(P*Bmax,2);
% 
% sigma_max = -1 + sqrt(1 + 1/PB_norm);
sigma = 2.5;


% ===================== ADDED STORAGE =====================
event_times  = [];
event_index  = [];
uf_hist      = zeros(2,length(t));
uhat_hist    = zeros(2,length(t));
% ========================================================

for k = 1:length(t)

    e = [cos(q(3)), sin(q(3)), 0; ...
        -sin(q(3)), cos(q(3)), 0; ...
         0,         0,         1]*(qRef(:,k) - q);
    e(3) = wrapToPi(e(3));

    vRef = uRef(1,k);
    wRef = uRef(2,k);

    Kx = 2*zeta*sqrt(wRef^2+g*vRef^2);
    Kphi = Kx;
    Ky = g*vRef;

    A = [1, Ts*wRef, 0;
        -Ts*wRef, 1, Ts*vRef;
         0, 0, 1];

    B = [-Ts, 0;
          0, 0;
          0, -Ts];

    uR = [vRef*cos(e(3)); wRef];
    K = [Kx 0 0; 0 Ky Kx];
    uB = K*e;

    % ===================== NORM-BASED TRIGGER =====================
    if norm(uhatB - uB,2) >= sigma*norm(e,2)
        uhatB = uB;
        event = event + 1,
        event_times(end+1) = t(k);
        event_index(end+1) = k;
    end
    % =============================================================

    u = uR + uhatB;

    wmr.setPose(q);
    ref.setPose(qRef(:,k));
    sige.plot(t(k), [0, 0, 0], e);
    sigU.plot(t(k), uR, u);
    drawnow limitrate nocallbacks;
    legend([wmr.oPath() ref.oPath()],{'Actual','desired'});
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

    dq = [u(1)*cos(q(3)); u(1)*sin(q(3)); u(2)];
    q = q + Ts*dq;
    q(3) = wrapToPi(q(3));
end

OFig.pause(Ts);
sigU.fin();
sige.fin();

%% ---- 3) Inter-event time ----
figure;
dt_event = diff(event_times);
stem(event_times(2:end), dt_event,'LineWidth',1.5);
xlabel('$t$ (s)','Interpreter','latex');
ylabel('$\Delta t_k$ (s)','Interpreter','latex');
grid on;

disp(['Total Events = ', num2str(event)])
