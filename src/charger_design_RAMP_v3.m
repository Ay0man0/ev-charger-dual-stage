%% =====================================================================
%  DUAL-STAGE EV CHARGER  -  design + simulation + power/efficiency report
%  Inputs : grid line-to-line RMS voltage and required charging power.
%  Does   : 1) calculates every design variable and prints each one,
%              with its formula, in the Command Window;
%           2) writes the variables SYSTEM_MODEL_param_RAMP.slx reads;
%           3) runs the model with logging of grid, DC-link and battery
%              signals, then plots input power vs output power and
%              calculates the efficiency of each stage.
%  Battery: the model's Battery block is a fixed 400 V / 250 Ah (100 kWh,
%           2C = 500 A) pack, so power is limited to 200 kW.
%% =====================================================================
clear; clc; close all;
modelName = 'SYSTEM_MODEL_param_RAMP_SOC';
pr  = @(name,val,unit,note) fprintf('  %-26s = %12.5g %-5s %s\n',name,val,unit,note);
sec = @(txt) fprintf('\n--- %s ---\n',txt);

%% 1. Inputs (press Enter for the default)
V_grid = input('Grid line-to-line RMS voltage [V]  (Enter = 400): ');
if isempty(V_grid), V_grid = 400; end
P_kW   = input('Required charging power [kW]        (Enter = 200): ');
if isempty(P_kW),   P_kW   = 200; end
assert(isscalar(V_grid) && V_grid>0 && isscalar(P_kW) && P_kW>0, 'Inputs must be positive numbers.');
SOC0 = input('Initial battery SOC [%]              (Enter = 70): ');
if isempty(SOC0), SOC0 = 70; end
assert(isscalar(SOC0) && SOC0>=0 && SOC0<100, 'Initial SOC must be between 0 and 100 %.');
if SOC0 > 90
    warning('Initial SOC above 90 percent: the battery voltage is close to the 447 V CV target, so the charging current will be limited.');
end
Vbat = 400;                                  % battery block nominal voltage [V]
if P_kW > 200
    warning('Battery pack in the model is 100 kWh / 2C = 200 kW max. Limiting power to 200 kW.');
    P_kW = 200;
end
P = P_kW*1e3;

%% 2. Fixed design assumptions (same as the original report)
Fs=50; Ws=2*pi*Fs; Fsw=10e3; Wsw=2*pi*Fsw; Ta=0.5/Fsw;
x=0.05; attenuation=0.05; R_per_H=37;
fsw=100e3; ripple_current=0.10; ripple_voltage=0.01; Vo_max=1.25*Vbat;

fprintf('\n=========== DESIGN CALCULATIONS: %g V grid, %g kW ===========\n',V_grid,P_kW);
sec('1. INPUTS AND ASSUMPTIONS');
pr('V_grid',V_grid,'V','grid line-to-line RMS (input)');
pr('P',P,'W','required charging power (input)');
pr('Vbat',Vbat,'V','battery nominal voltage (Battery block)');
pr('SOC0',SOC0,'%','initial battery state of charge (input, Battery block)');
pr('Fs',Fs,'Hz','grid frequency');
pr('Fsw',Fsw,'Hz','AFE switching frequency');
pr('fsw',fsw,'Hz','buck-boost switching frequency');
pr('I_line_rms',P/(sqrt(3)*V_grid),'A','P / (sqrt(3) V_grid)');

%% 3. AFE rectifier and LCL filter
sec('2. AFE RECTIFIER AND LCL FILTER');
Vdc_min = sqrt(2)*V_grid;
Vdc     = ceil(1.20*Vdc_min/50)*50;
Cdc     = P/(240*(0.1*Vdc)*(V_grid/sqrt(3))*Fs);
I_ripple= 0.2*sqrt(2)*P/(sqrt(3)*V_grid);
Zb=V_grid^2/P; Cb=1/(Ws*Zb);
L  = Vdc/(8*Fsw*I_ripple);
Cf = x*Cb;
a  = L*Cb*Wsw^2;
r  = (1-1/attenuation)/-(1-a*x);
if r<0, r=(1+1/attenuation)/-(1-a*x); end
Lf = L*r;  Lt = L+Lf;
Wres = sqrt(Lt/(L*Lf*Cf)); Fres = Wres/(2*pi);
rd_design = 1/(3*Wres*Cf);
R = R_per_H*L;  Rf = R_per_H*Lf;  Rt = R+Rf;
pr('Vdc_min',Vdc_min,'V','sqrt(2) V_grid');
pr('Vdc',Vdc,'V','ceil(1.2 Vdc_min / 50) * 50');
pr('Cdc',Cdc*1e6,'uF','P / (240 dV Vph Fs), dV = 10 % of Vdc');
pr('I_ripple',I_ripple,'A','0.2 sqrt(2) P / (sqrt(3) V_grid)');
pr('Zb',Zb,'Ohm','V_grid^2 / P');
pr('Cb',Cb*1e3,'mF','1 / (Ws Zb)');
pr('L',L*1e6,'uH','Vdc / (8 Fsw I_ripple)');
pr('Cf',Cf*1e6,'uF','x Cb, x = 0.05');
pr('a',a,'','L Cb Wsw^2');
pr('r',r,'','from attenuation factor 5 %');
pr('Lf',Lf*1e6,'uH','r L');
pr('Lt',Lt*1e6,'uH','L + Lf');
pr('Wres',Wres,'rad/s','sqrt(Lt / (L Lf Cf))');
pr('Fres',Fres/1e3,'kHz','Wres / (2 pi)');
pr('rd_design',rd_design,'Ohm','1 / (3 Wres Cf)  (model keeps 2 Ohm)');
pr('R',R,'Ohm','37 L');
pr('Rf',Rf,'Ohm','37 Lf');
pr('Rt',Rt,'Ohm','R + Rf');

%% 4. AFE controller gains
sec('3. AFE CONTROLLER');
fc  = min(Fsw/10, Fres/2.5);  wc=2*pi*fc;  wi=wc/10;
kpi = abs(1+1j*Ta*wc)*abs(Rt+1j*wc*Lt)/abs(1-1j*wi/wc);  kii = wi*kpi;
fc2 = 150;  wc2=2*pi*fc2;  wi2=wc2/5;
Kdc = 1.5*(V_grid/Vdc);
kpv = wc2*Cdc/(Kdc*abs(1-1j*wi2/wc2));  kiv = wi2*kpv;
Kmod = 1000/(Vdc/2);
m_index = sqrt(2/3)*V_grid/(Vdc/2);
pr('Ta',Ta*1e6,'us','1 / (2 Fsw)');
pr('fc (current loop)',fc,'Hz','min(Fsw/10, Fres/2.5)');
pr('wi (PI zero)',wi,'rad/s','wc / 10');
pr('kpi',kpi,'V/A','|1+jTa wc| |Rt+j wc Lt| / |1-j wi/wc|');
pr('kii',kii,'','wi kpi');
pr('fc2 (voltage loop)',fc2,'Hz','chosen');
pr('wi2 (PI zero)',wi2,'rad/s','wc2 / 5');
pr('Kdc',Kdc,'','1.5 V_grid / Vdc');
pr('kpv',kpv,'','wc2 Cdc / (Kdc |1-j wi2/wc2|)');
pr('kiv',kiv,'','wi2 kpv');
pr('Kmod',Kmod,'','1000 / (Vdc/2), carrier is +/-1000');
pr('m_index',m_index,'','sqrt(2/3) V_grid / (Vdc/2)');

%% 5. Synchronous buck-boost
sec('4. SYNCHRONOUS BUCK-BOOST');
Io_max = P/Vbat;
D_max  = Vo_max/(Vo_max+Vdc);
I_L    = Io_max/(1-D_max);
dIL    = ripple_current*I_L;
Lch_min= (Vdc*D_max)/(dIL*fsw);
Lch    = ceil(Lch_min/2e-6)*2e-6;
Co_min = (Io_max*D_max)/(fsw*ripple_voltage*Vo_max);
Co     = ceil(Co_min/100e-6)*100e-6;
IL_peak= I_L+0.5*dIL;
IL_SAT = 1.2*IL_peak;
Vsw_max= Vdc+Vo_max;
Dlim   = min(0.95, 1.13*D_max);
kpb    = 0.008     *(Lch/36e-6)*(1100/(Vdc+Vbat));
kib    = 3.0202337 *(Lch/36e-6)*(1100/(Vdc+Vbat));
Vcv    = 1.1175*Vbat;
pr('Vo_max',Vo_max,'V','1.25 Vbat');
pr('Io_max',Io_max,'A','P / Vbat');
pr('D_max',D_max,'','Vo_max / (Vo_max + Vdc)');
pr('I_L',I_L,'A','Io_max / (1 - D_max)');
pr('dIL',dIL,'A','10 % of I_L');
pr('Lch_min',Lch_min*1e6,'uH','Vdc D_max / (dIL fsw)');
pr('Lch',Lch*1e6,'uH','rounded up to 2 uH');
pr('Co_min',Co_min*1e6,'uF','Io_max D_max / (fsw dVo), dVo = 1 %');
pr('Co',Co*1e6,'uF','rounded up to 100 uF');
pr('IL_peak',IL_peak,'A','I_L + dIL/2');
pr('IL_SAT',IL_SAT,'A','1.2 IL_peak');
pr('Vsw_max',Vsw_max,'V','Vdc + Vo_max');
pr('Dlim',Dlim,'','min(0.95, 1.13 D_max)');
pr('kpb',kpb,'','0.008 (Lch/36u) (1100/(Vdc+Vbat))');
pr('kib',kib,'','3.02 (Lch/36u) (1100/(Vdc+Vbat))');
pr('Vcv',Vcv,'V','1.1175 Vbat (constant-voltage target)');

%% 6. Checks
sec('5. DESIGN CHECKS');
flags = {'** CHECK **','OK'};
fprintf('  6Fs < Fres < 0.5Fsw        : %s\n', flags{double(6*Fs<Fres && Fres<0.5*Fsw)+1});
fprintf('  Modulation index < 0.95    : %s (m = %.2f)\n', flags{double(m_index<0.95)+1}, m_index);
if Vsw_max > 0.75*1200 || IL_peak > 0.8*600
    fprintf('  NOTE: switch stress (%.0f V, %.0f A) exceeds a single 1200 V / 600 A module at 75/80 %% utilisation.\n',Vsw_max,IL_peak);
end
fprintf('\nWorkspace variables written for the model: V_grid SOC0 Vdc Cdc L Lf Cf R Rf Lt Fsw\n');
fprintf('  kpi kii kpv kiv Kmod Io_max Lch Co Dlim kpb kib Vcv\n');

%% 7. Run the model and calculate power / efficiency
if ~exist([modelName '.slx'],'file')
    disp(['Put ' modelName '.slx in the current folder, then run this script again.']); return;
end
if strcmpi(input('Run the simulation now? (y/n, Enter = y): ','s'),'n'), return; end
T_sim = input('Simulation stop time [s]  (Enter = 0.5): ');
if isempty(T_sim), T_sim = 0.5; end

bdclose all; load_system(modelName);
sigs = {'Vabc','Iabc','Vdc','Idc','Vbuck','Ibuck','SOC'};
% remove logging blocks left in the model by an earlier run (if any)
old = find_system(modelName,'SearchDepth',1,'regexp','on','Name','^LOG_');
for k = 1:numel(old)
    try
        lh = get_param(old{k},'LineHandles');
        fn = fieldnames(lh);
        for q = 1:numel(fn)
            hh = lh.(fn{q});
            for z = 1:numel(hh)
                if hh(z) > 0 && ishandle(hh(z)), delete_line(hh(z)); end
            end
        end
        delete_block(old{k});
    catch
    end
end
try
    for k = 1:numel(sigs)
        y = 80 + 50*k;
        add_block('simulink/Signal Routing/From',[modelName '/LOG_From_' sigs{k}], ...
            'GotoTag',sigs{k},'Position',[1000 y 1060 y+20]);
        add_block('simulink/Sinks/To Workspace',[modelName '/LOG_' sigs{k}], ...
            'VariableName',['log_' sigs{k}],'SaveFormat','Timeseries', ...
            'MaxDataPoints','inf','Position',[1120 y 1200 y+20]);
        add_line(modelName,['LOG_From_' sigs{k} '/1'],['LOG_' sigs{k} '/1']);
    end
    set_param(modelName,'StopTime',num2str(T_sim));
    open_system(modelName);
    simOut = sim(modelName,'ReturnWorkspaceOutputs','on');
catch ME
    disp('Simulation or logging failed:'); disp(ME.message); return;
end
disp('Simulation done. Calculating power and efficiency...');

dtu = 20e-6;                                   % uniform analysis step
ts0 = simOut.get('log_Vabc');
t0  = ts0.Time(:);
tu  = (min(t0):dtu:max(t0))';
D   = struct();
for k = 1:numel(sigs)
    ts = simOut.get(['log_' sigs{k}]);
    d  = squeeze(ts.Data);
    if size(d,1) ~= numel(ts.Time), d = d.'; end
    [tk,ik] = unique(ts.Time(:));
    D.(sigs{k}) = interp1(tk, d(ik,:), tu, 'linear', 'extrap');
end
Nw   = round(0.02/dtu);                        % one grid period moving average
Pin  = movmean(sum(D.Vabc.*D.Iabc,2), Nw);     % grid input power
Pdc  = movmean(D.Vdc.*D.Idc,          Nw);     % power leaving the AFE bridge
Pout = movmean(D.Vbuck.*D.Ibuck,      Nw);     % power into the battery

if T_sim >= 0.4, w1 = 0.25; w2 = 0.38; else, w1 = 0.6*T_sim; w2 = T_sim; end
idx = tu >= w1 & tu <= w2;
Pin_w = mean(Pin(idx)); Pdc_w = mean(Pdc(idx)); Pout_w = mean(Pout(idx));
eta_afe = Pdc_w/Pin_w; eta_dcdc = Pout_w/Pdc_w; eta_tot = Pout_w/Pin_w;
idx2 = tu >= 0.05;
eta_energy = trapz(tu(idx2),Pout(idx2))/trapz(tu(idx2),Pin(idx2));

fprintf('\n=========== POWER AND EFFICIENCY (window %.2f-%.2f s) ===========\n',w1,w2);
pr('Input power (grid)',Pin_w/1e3,'kW','mean of v.i over the window');
pr('DC-link power',Pdc_w/1e3,'kW','Vdc . Idc at the rectifier output');
pr('Output power (battery)',Pout_w/1e3,'kW','Vbuck . Ibuck');
pr('AFE + LCL efficiency',100*eta_afe,'%','Pdc / Pin');
pr('Buck-boost efficiency',100*eta_dcdc,'%','Pout / Pdc');
pr('Overall efficiency',100*eta_tot,'%','Pout / Pin');
pr('AFE + LCL loss',(Pin_w-Pdc_w)/1e3,'kW','Pin - Pdc');
pr('Buck-boost loss',(Pdc_w-Pout_w)/1e3,'kW','Pdc - Pout');
pr('Energy efficiency (>0.05 s)',100*eta_energy,'%','integral Pout / integral Pin');
fprintf('  Note: the efficiency covers the simulated components (switch and diode drops,\n  filter and inductor resistance, damping branch). Battery internal loss is excluded.\n');
%% Grid-side power factor over a whole number of grid periods in the same window
nper = floor((w2-w1)*Fs);  if nper < 1, nper = 1; end
ip   = tu >= w1 & tu < w1 + nper/Fs;
tt   = tu(ip);  vv = D.Vabc(ip,:);  ia = D.Iabc(ip,:);
P3   = mean(sum(vv.*ia,2));                    % real power [W]
Vrms = sqrt(mean(vv.^2,1));                    % rms of each phase voltage
Irms = sqrt(mean(ia.^2,1));                    % rms of each phase current
S3   = sum(Vrms.*Irms);                        % apparent power [VA]
PF   = P3/S3;                                  % true power factor (includes distortion)
ee   = exp(-1j*2*pi*Fs*tt);
V1   = 2*mean(vv.*ee,1);                       % fundamental phasors (peak) per phase
I1   = 2*mean(ia.*ee,1);
phi  = mean(angle(V1.*conj(I1)));              % > 0 means current lags voltage
DPF  = cos(phi);                               % displacement power factor
I1rms= abs(I1)/sqrt(2);
THDi = 100*mean(sqrt(max(Irms.^2 - I1rms.^2,0))./I1rms);
fprintf('\n=========== GRID-SIDE POWER FACTOR (window %.2f-%.2f s, %d grid periods) ===========\n',w1,w1+nper/Fs,nper);
pr('Real power P',P3/1e3,'kW','mean of v.i');
pr('Apparent power S',S3/1e3,'kVA','sum of Vrms . Irms over the 3 phases');
pr('True power factor',PF,'','P / S');
pr('Displacement power factor',DPF,'','cos(phase angle of the 50 Hz components)');
pr('Voltage-current angle',phi*180/pi,'deg','> 0 means current lags voltage');
pr('Fundamental current (rms)',mean(I1rms),'A','50 Hz component, mean of 3 phases');
pr('Current distortion (THD)',THDi,'%','includes the switching ripple');
fprintf('\n=========== BATTERY STATE OF CHARGE ===========\n');
pr('SOC at start',D.SOC(1),'%','from the Battery block');
pr('SOC at end',D.SOC(end),'%','at the stop time');
pr('SOC change',D.SOC(end)-D.SOC(1),'%','end minus start');
eff = struct('window',[w1 w2],'Pin_kW',Pin_w/1e3,'Pdc_kW',Pdc_w/1e3,'Pout_kW',Pout_w/1e3, ...
             'eta_afe',eta_afe,'eta_dcdc',eta_dcdc,'eta_total',eta_tot,'eta_energy',eta_energy,'PF',PF,'DPF',DPF); %#ok<NASGU>

fig = figure('Name','Input vs output power','Color','w','Position',[80 60 1000 820]);
subplot(3,1,1);
plot(tu,Pin/1e3,'b',tu,Pdc/1e3,'g',tu,Pout/1e3,'r','LineWidth',1.5); grid on;
legend('P_{in} (grid)','P_{DC link}','P_{out} (battery)','Location','best');
ylabel('Power [kW]'); title(sprintf('Input vs output power (%g V, %g kW), 20 ms moving average',V_grid,P_kW));
subplot(3,1,2);
plot(tu,(Pin-Pdc)/1e3,'g',tu,(Pdc-Pout)/1e3,'r',tu,(Pin-Pout)/1e3,'k','LineWidth',1.3); grid on;
legend('AFE + LCL loss','Buck-boost loss','Total loss','Location','best'); ylabel('Loss [kW]');
subplot(3,1,3);
mask = Pin > 0.05*P;
e1 = 100*Pdc./Pin;  e1(~mask) = NaN;
e2 = 100*Pout./Pdc; e2(~mask) = NaN;
e3 = 100*Pout./Pin; e3(~mask) = NaN;
plot(tu,e1,'g',tu,e2,'r',tu,e3,'k','LineWidth',1.3); grid on; ylim([60 110]);
legend('AFE + LCL','Buck-boost','Overall','Location','best');
ylabel('Efficiency [%]'); xlabel('Time [s]');
title('Efficiency vs time (values above 100 % are transients: stored inductor and capacitor energy)');
try, exportgraphics(fig,'power_efficiency.png','Resolution',200); catch, saveas(fig,'power_efficiency.png'); end
disp('Saved plot: power_efficiency.png');
disp('Also check the model scopes: the SOC should rise above its initial value.');
