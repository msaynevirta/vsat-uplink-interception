%% 
% Environment initialisation

addpath("matlab2tikz/src/");
%% 
% Physical constants

G = 6.673e-11; % universal gravitational constant, m3 * kg-1 * s-2
M_E = 5.9722e24 ; % mass of the Earth, kg
R = 6371008.8; % radius of the Earth, m
Gravitational_parameter_E = 3.986004418e14;
Omega_E = 2*pi / 86400; % earth's rotation in rad / s

Omega_E_vector = [0;0;Omega_E]; % angular velocity vector for Earth's rotation in the ECI coordinate system
%% 
% Effect of observation altitude to the range of a eavesdropping sensor in relation 
% to different elevation angles of the terminal.

d = @(h,ele)(h ./ (tand(ele)) / 1000); % maximum range in kilometers
%% 
% Let's plot d for elevation range of 10 deg to 90 deg and observation altitude 
% in range of 100 meters to 20 km.

ele = 10:2.5:85; % elevation angle in degrees
h = 0:750:20000; % aircraft altitude in meters

[H, E] = meshgrid(h, ele);

s = surf(E, H, d(H, E));
view(45,30);
ax = ancestor(s, 'axes');
ax.YAxis.Exponent = 0;
ax.ZAxis.Exponent = 0;
%ax.YLim = [0,20000];
%ax.XLim = [5,95];

xlabel('e (deg)');
ylabel('h (m)');
zlabel('d (km)');
matlab2tikz('doc/tikz_figures/fig-interception-range.tikz');
contourf(H,E,d(H,E));
%% 
% Orbital radius in relation to velocity of the sub-satellite point (in km/s), 
% ECI coordinate frame.

v_air = @(h,r_orbit)( (h+R) .* sqrt( (G*M_E) ./ (r_orbit + R).^3 ) );
%% 
% Plot the velocity for orbital radius in range of 500 km to 37000 km.

r_orbit = 500e3:500e3:37e6;

[H_air, R_orbit] = meshgrid(h, r_orbit);

p_v_air = plot(r_orbit, v_air(10000,r_orbit));
xlabel('h (m)');
ylabel('v (m/s)');

matlab2tikz('doc/tikz_figures/fig-subsat-velocity-equatiorial.tikz');
%% 
% In the case of GEO, Earth's rotation cancels the velocity of the sub-satellite 
% point. Next, we will evaluate the effect of orbital period and inclination to 
% the velocity of the sub-satellite point. We setup the orbits in the ECI coordinate 
% system and account for the Earth's rotation by converting to ECF.

inclination = 0:0.04:pi/2; % inclination in rad
orbital_period = 5400:2000:86400; % in seconds

roty = @(t)[cos(t) 0 sin(t); 0 1 0; -sin(t) 0 cos(t)] ;

omega_ECF_magnitude = @(i, T)( norm(roty(i) * ( (2*pi / T) .* [0;0;1] ) - Omega_E_vector));

[I_orbit, T_orbit] = meshgrid(inclination, orbital_period);

I_orbit_deg = I_orbit * (180 / pi);

omega_arr = arrayfun(omega_ECF_magnitude, I_orbit, T_orbit);

v_air_arr = R .* omega_arr;

orbital_altitude = (((Gravitational_parameter_E * ( T_orbit/(2*pi)).^2).^(1/3)) - R);

s_omega_ECF_norm = surf(I_orbit_deg, orbital_altitude, v_air_arr);

xlabel('i (deg)'); % inclination
ylabel('h (m)'); % orbital altitude
zlabel('v (m/s)'); % sub-satellite velocity 
view(-135,10);

matlab2tikz('doc/tikz_figures/fig-subsat-velocity-inclined.tikz');
%% 
% Listening window at a set beamwidth.

orbital_period = 5400:1000:42400; % in seconds 

beamwidth = deg2rad(3.5); % beamwidth in rad

roty = @(t)[cos(t) 0 sin(t); 0 1 0; -sin(t) 0 cos(t)] ;

[I_orbit, T_orbit] = meshgrid(inclination, orbital_period);

I_orbit_deg = I_orbit * (180 / pi);

omega_arr = arrayfun(omega_ECF_magnitude, I_orbit, T_orbit);

t_pass = beamwidth ./ omega_arr; % listening window, fixed aircraft

orbital_altitude = (((Gravitational_parameter_E * ( T_orbit/(2*pi)).^2).^(1/3)) - R);

s_omega_ECF_norm = surf(I_orbit_deg, orbital_altitude, t_pass);

xlabel('i (deg)'); % inclination
ylabel('h (m)'); % orbital altitude
zlabel('t (s)'); % maximum listening window / satellite pass
view([-318 10]);

matlab2tikz('doc/tikz_figures/fig-listening-window.tikz');
%% 
% OneWeb visualisation

%startTime=datetime("28-Feb-2023 20:30:00");
%endTime=datetime("28-Feb-2023 21:32:25");

startTime=datetime("11-Dec-2023 11:00:00"); % oneweb 2023-12-11
endTime=datetime("11-Dec-2023 11:30:00"); % oneweb 2023-12-11

%startTime=datetime("11-Dec-2023 11:00:00"); % starlink 2023-12-11
%endTime=datetime("11-Dec-2023 11:30:00"); % starlink 2023-12-11
sampleTime=60;

sc = satelliteScenario(startTime, endTime, sampleTime);

% 1200 km Earth orbit, 86.4 deg inclination, 588 sats in 12 planes. Phasing?
%ow = walkerStar(sc, 1200e3+6378.14e3, 86.4, 588, 12, 1, Name="OneWeb");

%utKajaani = groundStation(sc, 64.2, 27.7, Name="Kajaani");
%serverEspoo = groundStation(sc, 60.2, 24.8, Name="Espoo");
%serverWormerveer = groundStation(sc, 52.5, 4.8, Name="Wormerveer");
%popLondon = groundStation(sc, 51.5, -0.2, Name="London");

%set(utKajaani, ShowLabel=false);
%set(utKajaani, MarkerColor="Red");
%set(serverEspoo, ShowLabel=false);
%set(serverEspoo, MarkerColor="Green");
%set(serverWormerveer, ShowLabel=false);
%set(serverWormerveer, MarkerColor="Green");
%set(popLondon, ShowLabel=false);
%set(popLondon, MarkerColor="Yellow");

constellation = satellite(sc, "oneweb.tle");
%constellation = satellite(sc, "starlink.tle");

set(constellation(1:length(constellation)), ShowLabel=false);
set(constellation(1:length(constellation)), MarkerSize=3);
set(constellation(1:length(constellation)).Orbit, LineWidth=1);
%set(constellation(1:length(constellation)).Orbit, LineColor="none");

viewer3D = satelliteScenarioViewer(sc, Basemap="bluegreen", ShowDetails=true);
%%
campos(viewer3D, 40, 18);